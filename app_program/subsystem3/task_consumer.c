/*
 *	task_consumer.c
 *	Subsystem 3 -- Consumer task combined with IR sensors, Code 39 decoding, and Subsystem 2 motor control.
 */

#include "subsystem3.h"
#include "../app_debug.h"
#include "../subsystem2/subsystem2.h"   /* IF-01/02: SAMPLE_MSG */
#include "../subsystem2/motion_control.h"   /* Subsystem 2 Motor Control API */
#include "../subsystem4/subsystem4.h"   /* IF-03: mbf_summary */
#include "../vehicle_controller/vehicle_controller.h" /* passive line-intent bridge */

#define LINE_RECOVERY_ARC_MS 500u
#define RIGHT_LINE_LOSS_ARC_MS 150u
#define LINE_THRESHOLD_HYSTERESIS 100u

/* Barcode is sampled from the ADC (channel 2 / GP28 AO) in the consumer
 * loop.  Each analog sample is turned into a black/white level using an
 * adaptive midpoint between the learned white and black extremes, and a
 * level change is treated as an edge, timed from the hardware microsecond
 * timer, then fed into the same width-capture and decode path as before. */
typedef struct {
	UW us;
	bool high;
} barcode_edge_t;

static UW high_bar_widths[BARCODE_BAR_COUNT], low_bar_widths[BARCODE_BAR_COUNT];
static UW high_bar_count, low_bar_count, last_edge_time, first_edge_time;
static bool scanning, barcode_level_high;

/* Adaptive black/white decision for the analog barcode signal.
 * BAR_IS_HIGH: a black bar reads HIGHER than the white floor, so
 * barcode_black_level tracks the maximum and barcode_white_level the min. */
static bool barcode_level_init;
static UW barcode_white_level;
static UW barcode_black_level;

/* Decoded barcode character waiting to be turned into a motion command.
 * finish_barcode() runs inside the edge-processing path, so it only records
 * the result here; the task loop consumes it and posts the command through
 * the vehicle controller (keeping safety/obstacle arbitration in the loop).
 * 0 = nothing pending. */
static volatile char barcode_pending_cmd = 0;

/*
 * Full Code 39 decoder (A-Z, 0-9 and the standard symbols).
 *
 * A Code 39 character is nine elements, five bars and four spaces, read
 * bar,space,bar,space,bar,space,bar,space,bar.  Exactly three of the nine
 * are wide ("3 of 9").  The table below holds each character's 9-bit
 * pattern, MSB = first element, 1 = wide, 0 = narrow.  '*' (0x094) frames
 * the data as the start and stop character.
 *
 * A scanned "*X*" barcode is 15 bars + 14 spaces = 29 strictly alternating
 * elements, starting and ending on a bar.  They arrive here as two arrays:
 *   high_bar_widths[0..14]  (the black bars)
 *   low_bar_widths[0..13]   (the white spaces)
 * interleaved as high[0], low[0], high[1], low[1], ... high[14].
 * Three 9-element groups (offsets 0, 10, 20) with a 1-element inter-character
 * gap between them form "* data *".
 */
typedef struct { char ch; UW pat; } code39_entry_t;
static const code39_entry_t code39_table[] = {
	{'0',0x034},{'1',0x121},{'2',0x061},{'3',0x160},{'4',0x031},
	{'5',0x130},{'6',0x070},{'7',0x025},{'8',0x124},{'9',0x064},
	{'A',0x109},{'B',0x049},{'C',0x148},{'D',0x019},{'E',0x118},
	{'F',0x058},{'G',0x00D},{'H',0x10C},{'I',0x04C},{'J',0x01C},
	{'K',0x103},{'L',0x043},{'M',0x142},{'N',0x013},{'O',0x112},
	{'P',0x052},{'Q',0x007},{'R',0x106},{'S',0x046},{'T',0x016},
	{'U',0x181},{'V',0x0C1},{'W',0x1C0},{'X',0x091},{'Y',0x190},
	{'Z',0x0D0},{'-',0x085},{'.',0x184},{' ',0x0C4},{'$',0x0A8},
	{'/',0x0A2},{'+',0x08A},{'%',0x02A},{'*',0x094}
};

/* Classify nine elements (starting at `elements[offset]`) into a 9-bit
 * narrow/wide pattern, then look it up.  `wide_threshold` is shared across
 * the whole barcode so one narrow/wide boundary is used throughout. */
static char code39_lookup(const UW *elements, UW offset, UW wide_threshold)
{
	UW pattern = 0u;
	UW wide_count = 0u;
	UW i;

	for (i = 0u; i < 9u; ++i) {
		pattern <<= 1;
		if (elements[offset + i] >= wide_threshold) {
			pattern |= 1u;
			++wide_count;
		}
	}
	if (wide_count != 3u) {     /* every valid Code 39 char has 3 wide */
		return '?';
	}
	for (i = 0u; i < sizeof(code39_table) / sizeof(code39_table[0]); ++i) {
		if (code39_table[i].pat == pattern) {
			return code39_table[i].ch;
		}
	}
	return '?';
}

/* Decode one "*X*" scan into its middle character.  Returns '?' on failure.
 * bar_widths = 15 black bars, space_widths = 14 white spaces. */
static char decode_barcode(const UW *bar_widths, const UW *space_widths)
{
	UW elements[29];
	UW sorted[29];
	UW i, j;
	UW narrow_total = 0u;
	UW narrow_avg;
	UW wide_threshold;
	char start_c, data_c, stop_c;

	/* Interleave bars and spaces into the real scan order (29 elements). */
	for (i = 0u; i < BARCODE_BAR_COUNT; ++i) {
		elements[2u * i] = bar_widths[i];              /* bar  */
		if (i < BARCODE_BAR_COUNT - 1u) {
			elements[2u * i + 1u] = space_widths[i];   /* space */
		}
	}

	/* Narrow reference: of 29 elements, Code 39 "*X*" has more narrow than
	 * wide, so average the narrowest ~17 and set the wide cutoff at 1.5x. */
	for (i = 0u; i < 29u; ++i) {
		UW w = elements[i];
		j = i;
		while (j > 0u && sorted[j - 1u] > w) { sorted[j] = sorted[j - 1u]; --j; }
		sorted[j] = w;
	}
	for (i = 0u; i < 17u; ++i) {
		narrow_total += sorted[i];
	}
	narrow_avg = narrow_total / 17u;
	if (narrow_avg == 0u) {
		return '?';
	}
	wide_threshold = narrow_avg + narrow_avg / 2u;   /* 1.5 x narrow */

	/* Groups: [0..8]=start, [9]=gap, [10..18]=data, [19]=gap, [20..28]=stop. */
	start_c = code39_lookup(elements, 0u, wide_threshold);
	data_c  = code39_lookup(elements, 10u, wide_threshold);
	stop_c  = code39_lookup(elements, 20u, wide_threshold);

	if (start_c == '*' && stop_c == '*' && data_c != '?' && data_c != '*') {
		return data_c;
	}
	return '?';
}

static void finish_barcode(void)
{
	char decoded = '?';
	/* A complete "*X*" scan is 15 black bars and 14 white spaces. */
	if (high_bar_count == BARCODE_BAR_COUNT &&
	    low_bar_count == BARCODE_BAR_COUNT - 1u) {
		decoded = decode_barcode(high_bar_widths, low_bar_widths);
		if (decoded == '?') {
			/* Try the reverse scan direction. */
			UW rbar[BARCODE_BAR_COUNT];
			UW rspace[BARCODE_BAR_COUNT - 1u];
			UW k;
			for (k = 0u; k < BARCODE_BAR_COUNT; ++k)
				rbar[k] = high_bar_widths[BARCODE_BAR_COUNT - 1u - k];
			for (k = 0u; k < BARCODE_BAR_COUNT - 1u; ++k)
				rspace[k] = low_bar_widths[BARCODE_BAR_COUNT - 2u - k];
			decoded = decode_barcode(rbar, rspace);
		}
	}
	if (decoded != '?') {
		tm_printf((UB*)"-> Decoded Character: %c\r\n", decoded);
		/* Hand the character to the task loop; only A-D will move the car. */
		barcode_pending_cmd = decoded;
	} else if (high_bar_count >= BARCODE_BAR_COUNT || low_bar_count >= BARCODE_BAR_COUNT) {
		tm_printf((UB*)"Unknown barcode pattern.\r\n");
	}
	scanning = false;
}

static void barcode_tick(UW now)
{
	if (!scanning) return;
	if (now - last_edge_time >= QUIET_ZONE_TIMEOUT_MS * 1000u)
		finish_barcode();
	else if (now - first_edge_time >= BARCODE_TIMEOUT_MS * 1000u)
		scanning = false;
}

static void consume_barcode_edge(barcode_edge_t edge)
{
	barcode_tick(edge.us);
	if (!scanning) {
		scanning = true;
		first_edge_time = edge.us;
		high_bar_count = low_bar_count = 0u;
	} else {
		UW width = edge.us - last_edge_time;
		if (edge.high == barcode_level_high || width < BARCODE_EDGE_DEBOUNCE_US)
			return;
		UW *count = barcode_level_high ? &high_bar_count : &low_bar_count;
		UW *widths = barcode_level_high ? high_bar_widths : low_bar_widths;
		if (*count < BARCODE_BAR_COUNT)
			widths[*count] = width;
		if (*count <= BARCODE_BAR_COUNT) ++*count;
	}
	last_edge_time = edge.us;
	barcode_level_high = edge.high;
}

static void poll_barcode(UW adc_value)
{
	UW now = in_w(TIMER_TIMERAWL);
	UW span, midpoint, deadband;
	bool next_level;

	/* First sample: assume the sensor sits over white, seed both levels. */
	if (!barcode_level_init) {
		barcode_white_level = adc_value;
		barcode_black_level = adc_value;
		barcode_level_high = false;
		barcode_level_init = true;
		barcode_tick(now);
		return;
	}

	/* Learn the extremes so the decision adapts to contrast/height. */
	if (adc_value < barcode_white_level) {
		barcode_white_level = adc_value;
	}
	if (adc_value > barcode_black_level) {
		barcode_black_level = adc_value;
	}

	/* Not enough contrast yet (over plain white): hold the level. */
	span = barcode_black_level - barcode_white_level;
	if (span < BARCODE_MIN_CONTRAST) {
		barcode_tick(now);
		return;
	}

	midpoint = barcode_white_level + span / 2u;
	deadband = (span * BARCODE_HYST_PCT) / 100u;

	next_level = barcode_level_high
		? (adc_value > midpoint - deadband)   /* stay black unless clearly below */
		: (adc_value >= midpoint + deadband); /* go black only if clearly above */

	if (next_level != barcode_level_high) {
		barcode_edge_t edge = { now, next_level };
		consume_barcode_edge(edge);
	} else {
		barcode_tick(now);
	}
}

static ER read_adc(ID device, W channel, UW *value)
{
	SZ size = 0;
	ER er = tk_srea_dev(device, channel, value, 1, &size);

	if (er != E_OK) {
		return er;
	}
	return (size == 1) ? E_OK : E_IO;
}

EXPORT void consumer_task(INT stacd, void *exinf)
{
	SAMPLE_MSG  *msg;
	UB      text[S4_SUMMARY_MAX];
	INT     len;
	UW      count;
	ER      er;

	(void)stacd; (void)exinf;

	ID adc_device = tk_opn_dev((const UB*)"adca", TD_READ);
	if (adc_device < E_OK) {
		tm_printf((UB*)"[S3] Failed to open ADC device: %d\r\n", adc_device);
		return;
	}

	RobotState current_state = STATE_FOLLOWING_LINE;
	SYSTIM current_time;
	er = tk_get_otm(&current_time);
	if (er != E_OK) {
		tm_printf((UB*)"[S3] Failed to read system time: %d\r\n", er);
		(void)tk_cls_dev(adc_device, 0);
		return;
	}
	UW line_lost_start_time = 0;
#if APP_DEBUG_S3_SENSOR
	UW last_debug_print = current_time.lo;
#endif
	UB last_line_motion_command = 0;
	bool right_line_correction_active = false;
	bool right_line = false;
	bool center_line = false;
	/* While non-zero, a barcode turn is executing: hold off line-follow
	 * steering until this deadline (ms) so the turn is not immediately
	 * overwritten by the per-cycle line intent. */
	UW barcode_turn_until = 0u;
	#define BARCODE_TURN_MS 700u
	/* Barcode is now sampled via ADC channel 2 in the loop; no GPIO IRQ. */

	while (1) {
		/* --- PART A: Non-blocking check for background telemetry/statistics samples --- */
		er = tk_rcv_mbx(mbx_samples, (T_MSG **)&msg, TMO_POL);
		if (er == E_OK && msg != NULL) {
			tk_loc_mtx(mtx_stats, TMO_FEVR);
			if (app_stats.received == 0 || msg->value < app_stats.min) {
				app_stats.min = msg->value;
			}
			if (app_stats.received == 0 || msg->value > app_stats.max) {
				app_stats.max = msg->value;
			}
			app_stats.sum += msg->value;
			count = ++app_stats.received;
			tk_unl_mtx(mtx_stats);

			if (count % S3_BATCH_SIZE == 0) {
				len = tm_sprintf(text, (const UB*)"n=%u last=%d from core %d",
						 count, msg->value, msg->prc - 1);
				if (tk_snd_mbf(mbf_summary, text, len, TMO_POL) == E_OK) {
					tk_set_flg(flg_events, FLG_BATCH);
				}
			}
			tk_rel_mpf(mpf_samples, msg);
			tk_sig_sem(sem_slots, 1);
		}

		/* --- PART B: IR Line Tracking, Barcode Reading, & Subsystem 2 Motor Control --- */
		UW right;
		UW center;
		UW barcode_adc = 0u;
		er = read_adc(adc_device, ADC_CHANNEL_RIGHT, &right);
		if (er == E_OK) {
			er = read_adc(adc_device, ADC_CHANNEL_CENTER, &center);
		}
		if (er == E_OK) {
			er = read_adc(adc_device, ADC_CHANNEL_BARCODE, &barcode_adc);
		}
		if (er != E_OK) {
			tm_printf((UB*)"[S3] ADC read failed: %d\r\n", er);
			(void)tk_cls_dev(adc_device, 0);
			return;
		}

		er = tk_get_otm(&current_time);
		if (er != E_OK) {
			tm_printf((UB*)"[S3] Failed to read system time: %d\r\n", er);
			(void)tk_cls_dev(adc_device, 0);
			return;
		}
		UW now = current_time.lo;

		right_line = right_line ?
			(right > THRESHOLD - LINE_THRESHOLD_HYSTERESIS) :
			(right > THRESHOLD + LINE_THRESHOLD_HYSTERESIS);
		center_line = center_line ?
			(center > THRESHOLD - LINE_THRESHOLD_HYSTERESIS) :
			(center > THRESHOLD + LINE_THRESHOLD_HYSTERESIS);
		
#if APP_DEBUG_S3_SENSOR
		/* Periodic raw ADC/barcode readout: optional during integration. */
		if ((UW)(now - last_debug_print) >= 1000) {
			UW bspan = (barcode_black_level >= barcode_white_level)
				? (barcode_black_level - barcode_white_level) : 0u;
			tm_printf((UB*)"RAW -> R: %4u [%s] | C: %4u [%s] | Barcode ADC2: %4u white=%u black=%u span=%u\r\n",
				      right, right_line ? "BLACK" : "WHITE",
				      center, center_line ? "BLACK" : "WHITE",
				      barcode_adc, barcode_white_level, barcode_black_level, bspan);
			last_debug_print = now;
		}
#endif

		poll_barcode(barcode_adc);

		/* Act on a freshly decoded barcode.  Any character is printed by
		 * finish_barcode(); only A-D map to a movement instruction, posted
		 * through the vehicle controller so safety/obstacle arbitration
		 * still applies.  A = turn left, B = turn right, C = straight,
		 * D = U-turn (two left pivots).  Other letters are ignored here. */
		{
			char bc = barcode_pending_cmd;
			if (bc != 0) {
				barcode_pending_cmd = 0;
				switch (bc) {
				case 'A':
					tm_printf((UB*)"[S3] Barcode A -> turn LEFT\r\n");
					(void)vehicle_controller_line_motion_post(MC_CMD_DRIVE_LEFT, FALSE);
					barcode_turn_until = now + BARCODE_TURN_MS;
					break;
				case 'B':
					tm_printf((UB*)"[S3] Barcode B -> turn RIGHT\r\n");
					(void)vehicle_controller_line_motion_post(MC_CMD_DRIVE_RIGHT, FALSE);
					barcode_turn_until = now + BARCODE_TURN_MS;
					break;
				case 'C':
					tm_printf((UB*)"[S3] Barcode C -> go STRAIGHT\r\n");
					(void)vehicle_controller_line_motion_post(MC_CMD_DRIVE_FORWARD, FALSE);
					/* Straight: no hold needed, line following continues. */
					break;
				case 'D':
					tm_printf((UB*)"[S3] Barcode D -> U-TURN\r\n");
					/* U-turn = a longer left pivot; line following reacquires
					 * the line afterwards. */
					(void)vehicle_controller_line_motion_post(MC_CMD_DRIVE_LEFT, FALSE);
					barcode_turn_until = now + 2u * BARCODE_TURN_MS;
					break;
				default:
					/* A-Z decoded and printed, but no movement for E-Z. */
					break;
				}
			}
		}

		/* While a barcode turn is executing, hold the turn and skip the
		 * line-follow steering so it is not overwritten each cycle. */
		if (barcode_turn_until != 0u) {
			if ((INT)(now - barcode_turn_until) < 0) {
				goto barcode_turn_hold;
			}
			barcode_turn_until = 0u;
			last_line_motion_command = 0;   /* force fresh line intent */
		}

		/* Resume steering on the same sample that reacquires the line. */
		if (current_state == STATE_LOST_LINE && (right_line || center_line))
			current_state = STATE_FOLLOWING_LINE;

		switch (current_state) {
			case STATE_FOLLOWING_LINE:
				if (!right_line && !center_line) {
					if (right_line_correction_active) {
#if APP_DEBUG_S3_STEERING
						tm_printf((UB*)"[STEERING] Right correction lost the line; trying a brief right arc.\r\n");
#endif
						line_lost_start_time = now;
						er = vehicle_controller_line_motion_post(MC_CMD_STEER_RIGHT, right_line || center_line);
						if (er == E_OK) {
							last_line_motion_command = MC_CMD_STEER_RIGHT;
						}
						current_state = STATE_LOST_LINE;
						break;
					}

#if APP_DEBUG_S3_STEERING
					tm_printf((UB*)"[STEERING] Line lost; starting gentle left recovery arc.\r\n");
#endif
					line_lost_start_time = now;
					er = vehicle_controller_line_motion_post(MC_CMD_STEER_LEFT, right_line || center_line);
					if (er == E_OK) {
						last_line_motion_command = MC_CMD_STEER_LEFT;
					}
					current_state = STATE_LOST_LINE;
					break;
				}

				UB line_motion_command = MC_CMD_DRIVE_FORWARD;
				if (center_line) {
					right_line_correction_active = false;
					line_motion_command = MC_CMD_DRIVE_FORWARD;
				} else if (right_line) {
					right_line_correction_active = true;
					line_motion_command = MC_CMD_STEER_RIGHT;
				} 

				{ /* Refresh VC intent while following line (stale timeout). */
					er = vehicle_controller_line_motion_post(line_motion_command, right_line || center_line);
					if (er == E_OK) {
						last_line_motion_command = line_motion_command;
					}
				}
				break;

			case STATE_LOST_LINE:
				if (right_line_correction_active &&
					   (UW)(now - line_lost_start_time) <
						   RIGHT_LINE_LOSS_ARC_MS) {
					{ /* Refresh VC recovery intent. */
						er = vehicle_controller_line_motion_post(MC_CMD_STEER_RIGHT, right_line || center_line);
						if (er == E_OK) {
							last_line_motion_command = MC_CMD_STEER_RIGHT;
						}
					}
				} else if (right_line_correction_active) {
					right_line_correction_active = false;
#if APP_DEBUG_S3_STEERING
					tm_printf((UB*)"[STEERING] Brief right arc did not reacquire the line; switching to left recovery.\r\n");
#endif
					line_lost_start_time = now;
					er = vehicle_controller_line_motion_post(MC_CMD_STEER_LEFT, right_line || center_line);
					if (er == E_OK) {
						last_line_motion_command = MC_CMD_STEER_LEFT;
					}
				} else if ((UW)(now - line_lost_start_time) >=
					   LINE_RECOVERY_ARC_MS &&
					   last_line_motion_command != MC_CMD_DRIVE_LEFT) {
#if APP_DEBUG_S3_STEERING
					tm_printf((UB*)"[STEERING] Line not found; escalating to sharper left pivot.\r\n");
#endif
					er = vehicle_controller_line_motion_post(MC_CMD_DRIVE_LEFT, right_line || center_line);
					if (er == E_OK) {
						last_line_motion_command = MC_CMD_DRIVE_LEFT;
					}
				}
				break;

			case STATE_TIMEOUT_RECOVERY:
				current_state = STATE_FOLLOWING_LINE;
				break;
		}

		/* Heartbeat: keep line-recovery intent fresh while searching.
		 * No actual motion commands are sent in Phase 3. */
		if (current_state == STATE_LOST_LINE && last_line_motion_command != 0u) {
			(void)vehicle_controller_line_motion_post(last_line_motion_command, right_line || center_line);
		}

	barcode_turn_hold:
		tk_dly_tsk(1);
	}
}
