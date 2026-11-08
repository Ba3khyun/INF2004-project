/*
 *	task_consumer.c
 *	Subsystem 3 -- Consumer task combined with IR sensors, Code 39 decoding, and Subsystem 2 motor control.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"   /* IF-01/02: SAMPLE_MSG */
#include "../subsystem2/motion_control.h"   /* Subsystem 2 Motor Control API */
#include "../subsystem4/subsystem4.h"   /* IF-03: mbf_summary */
#include "../vehicle_controller/vehicle_controller.h" /* passive line-intent bridge */

#define LINE_RECOVERY_ARC_MS 500u
#define RIGHT_LINE_LOSS_PIVOT_MS 150u
#define LINE_THRESHOLD_HYSTERESIS 100u

static volatile UW high_bar_widths[BARCODE_BAR_COUNT]; //when at black
static volatile UW low_bar_widths[BARCODE_BAR_COUNT]; //when at white
static volatile UW high_bar_count = 0;
static volatile UW low_bar_count = 0;
static volatile UW barcode_edge_count = 0;
static volatile UW barcode_raw_edge_count = 0;
static volatile UW barcode_ignored_edge_count = 0;
static volatile UW last_edge_time = 0;
static volatile bool barcode_scan_in_progress = false;
static volatile bool barcode_level_high = false;

static void capture_bar_width(volatile UW *widths, volatile UW *count, UW width)
{
	if (width < BARCODE_MIN_BAR_WIDTH_US) {
		return;
	}
	if (*count < BARCODE_BAR_COUNT) {
		widths[*count] = width;
	} else {
		for (UW i = 1u; i < BARCODE_BAR_COUNT; ++i) {
			widths[i - 1u] = widths[i];
		}
		widths[BARCODE_BAR_COUNT - 1u] = width;
	}
	if (*count < BARCODE_BAR_COUNT + 1u) {
		(*count)++;
	}
}

static void barcode_gpio_isr(UINT gpio, UINT events)
{
	UW low_edge = 1u << (4u * (gpio % 8u) + 2u);
	UW high_edge = 1u << (4u * (gpio % 8u) + 3u);
	UW now = in_w(TIMER_TIMERAWL);
	bool edge_level_high;

	if ((events & (low_edge | high_edge)) == 0u) {
		return;
	}
	barcode_raw_edge_count++;
	edge_level_high = gpio_get_val(gpio) != 0u;

	if (!barcode_scan_in_progress) {
		barcode_scan_in_progress = true;
		high_bar_count = 0;
		low_bar_count = 0;
		barcode_edge_count = 1;
		barcode_ignored_edge_count = 0;
		barcode_level_high = edge_level_high;
		last_edge_time = now;
		return;
	}

	if (edge_level_high == barcode_level_high ||
	    (now - last_edge_time) < BARCODE_EDGE_DEBOUNCE_US) {
		barcode_ignored_edge_count++;
		return;
	}

	if (barcode_level_high) {
		capture_bar_width(high_bar_widths, &high_bar_count, now - last_edge_time);
	} else {
		capture_bar_width(low_bar_widths, &low_bar_count, now - last_edge_time);
	}
	if (barcode_edge_count < 0xFFFFFFFFu) {
		barcode_edge_count++;
	}
	barcode_level_high = edge_level_high;
	last_edge_time = now;
}

static void reset_barcode_scan(void)
{
	barcode_scan_in_progress = false;
	high_bar_count = 0;
	low_bar_count = 0;
	barcode_edge_count = 0;
	barcode_raw_edge_count = 0;
	barcode_ignored_edge_count = 0;
}

static char decode_barcode(const volatile UW *bar_widths, UW *pattern_out)
{
	static const UW pattern_a = 0x1A26u; //NNWWNWNNNWNNWWN
	static const UW pattern_b = 0x1926u; //NNWWNNWNNWNNWWN
	static const UW pattern_c = 0x1B06u; //NNWWNWWNNNNNWWN
	static const UW pattern_d = 0x18A6u; //NNWWNNNWNWNNWWN
	UW sorted_widths[BARCODE_BAR_COUNT];
	UW narrow_total = 0u;
	UW narrow_average;
	UW wide_threshold;
	UW pattern = 0u;
	UW reversed_pattern = 0u;
	UW i;

	for (i = 0u; i < BARCODE_BAR_COUNT; ++i) {
		UW width = bar_widths[i];
		UW j = i;

		while (j > 0u && sorted_widths[j - 1u] > width) {
			sorted_widths[j] = sorted_widths[j - 1u];
			--j;
		}
		sorted_widths[j] = width;
	}

	/* These templates each contain nine narrow and six wide bars. */
	for (i = 0u; i < 9u; ++i) {
		narrow_total += sorted_widths[i];
	}
	narrow_average = narrow_total / 9u;
	if (narrow_average == 0u) {
		*pattern_out = 0u;
		return '?';
	}
	wide_threshold = narrow_average * 2u;

	for (i = 0u; i < BARCODE_BAR_COUNT; ++i) {
		pattern <<= 1;
		if (bar_widths[i] >= wide_threshold) {
			pattern |= 1u;
		}
	}
	*pattern_out = pattern;

	for (i = 0u; i < BARCODE_BAR_COUNT; ++i) {
		reversed_pattern = (reversed_pattern << 1) | (pattern & 1u);
		pattern >>= 1;
	}
	if (*pattern_out == pattern_a || reversed_pattern == pattern_a) return 'A';
	if (*pattern_out == pattern_b || reversed_pattern == pattern_b) return 'B';
	if (*pattern_out == pattern_c || reversed_pattern == pattern_c) return 'C';
	if (*pattern_out == pattern_d || reversed_pattern == pattern_d) return 'D';
	return '?';
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
	UW      irq_wait;
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
	UW barcode_start_time = 0;
	UW line_lost_start_time = 0;
	UW last_debug_print = current_time.lo;
	UB last_line_motion_command = 0;
	bool right_line_correction_active = false;
	bool right_line = false;
	bool center_line = false;
	for (irq_wait = 0u; irq_wait < 100u; ++irq_wait) {
		er = motion_gpio_irq_register(BARCODE_PIN, barcode_gpio_isr);
		if (er != E_BUSY) {
			break;
		}
		tk_dly_tsk(1);
	}
	if (er != E_OK) {
		tm_printf((UB*)"[S3] Failed to register barcode GPIO interrupt: %d\r\n", er);
		(void)tk_cls_dev(adc_device, 0);
		return;
	}

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
		er = read_adc(adc_device, ADC_CHANNEL_RIGHT, &right);
		if (er == E_OK) {
			er = read_adc(adc_device, ADC_CHANNEL_CENTER, &center);
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
		
		int barcode_val_print = gpio_get_val(BARCODE_PIN) != 0 ? 1 : 0;

		/// Print debug status once per second.
		if ((UW)(now - last_debug_print) >= 1000) {
			tm_printf((UB*)"RAW -> R: %4u [%s] | C: %4u [%s] | Barcode (GP2): %d | Threshold: %d\r\n",
				      right, right_line ? "BLACK" : "WHITE",
				      center, center_line ? "BLACK" : "WHITE",
				      barcode_val_print, THRESHOLD);
			last_debug_print = now;
		}

		// Transition into barcode reading state as soon as ISR flags activity
		if (barcode_scan_in_progress &&
		    (current_state == STATE_FOLLOWING_LINE ||
		     current_state == STATE_LOST_LINE)) {
			tm_printf((UB*)"Transitioning to STATE_READING_BARCODE...\r\n");
			barcode_start_time = in_w(TIMER_TIMERAWL);
			current_state = STATE_READING_BARCODE;
		}

		switch (current_state) {
			case STATE_FOLLOWING_LINE:
				if (!right_line && !center_line) {
					if (right_line_correction_active) {
						tm_printf((UB*)"[STEERING] Right correction lost the line; trying a brief sharp right pivot.\r\n");
						line_lost_start_time = now;
						er = vehicle_controller_line_motion_post(MC_CMD_DRIVE_RIGHT, right_line || center_line);
						if (er == E_OK) {
							last_line_motion_command = MC_CMD_DRIVE_RIGHT;
						}
						current_state = STATE_LOST_LINE;
						break;
					}

					tm_printf((UB*)"[STEERING] Line lost; starting gentle left recovery arc.\r\n");
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

			case STATE_READING_BARCODE:
			{
				UW current_us = in_w(TIMER_TIMERAWL);
				
				//Check how long it has been reading the barcode
				if ((current_us - barcode_start_time) > (BARCODE_TIMEOUT_MS * 1000UL)) {
					tm_printf((UB*)"TIMEOUT: Barcode read took too long. Raw edges=%u, accepted=%u, ignored=%u, high=%u bars, low=%u bars, GP2=%u.\r\n",
						  barcode_raw_edge_count, barcode_edge_count,
						  barcode_ignored_edge_count, high_bar_count, low_bar_count,
						  gpio_get_val(BARCODE_PIN));
					reset_barcode_scan();
					current_state = STATE_FOLLOWING_LINE; 
					break;
				}

				bool high_scan_complete =
					high_bar_count >= BARCODE_BAR_COUNT && !barcode_level_high;
				bool low_scan_complete =
					low_bar_count >= BARCODE_BAR_COUNT && barcode_level_high;
				if ((high_scan_complete || low_scan_complete) &&
				    (current_us - last_edge_time) >
					    (QUIET_ZONE_TIMEOUT_MS * 1000UL)) {
					UW high_pattern = 0u;
					UW low_pattern = 0u;
					char high_decoded = '?';
					char low_decoded = '?';
					char decoded_char;

					if (high_scan_complete) {
						high_decoded = decode_barcode(high_bar_widths, &high_pattern);
					}
					if (low_scan_complete) {
						low_decoded = decode_barcode(low_bar_widths, &low_pattern);
					}
					if (high_decoded != '?' && low_decoded != '?' &&
					    high_decoded != low_decoded) {
						decoded_char = '?';
					} else if (high_decoded != '?') {
						decoded_char = high_decoded;
					} else {
						decoded_char = low_decoded;
					}

					tm_printf((UB*)"Barcode scan complete. Raw edges=%u, accepted=%u, ignored=%u, high=%u bars, low=%u bars.\r\n",
						  barcode_raw_edge_count, barcode_edge_count,
						  barcode_ignored_edge_count, high_bar_count, low_bar_count);
					if (decoded_char == '?') {
						tm_printf((UB*)"Unknown or ambiguous barcode pattern (high=0x%x, low=0x%x).\r\n",
							  high_pattern, low_pattern);
					} else {
						tm_printf((UB*)"-> Decoded Character: %c\r\n", decoded_char);
					}

					reset_barcode_scan();
					current_state = STATE_FOLLOWING_LINE;
				}
				break;
			}

			case STATE_LOST_LINE:
				if (right_line || center_line) {
					right_line_correction_active = false;
					tm_printf((UB*)"[STEERING] Line reacquired; resuming line following.\r\n");
					last_line_motion_command = 0;
					current_state = STATE_FOLLOWING_LINE;
				} else if (right_line_correction_active &&
					   (UW)(now - line_lost_start_time) <
						   RIGHT_LINE_LOSS_PIVOT_MS) {
					{ /* Refresh VC recovery intent. */
						er = vehicle_controller_line_motion_post(MC_CMD_DRIVE_RIGHT, right_line || center_line);
						if (er == E_OK) {
							last_line_motion_command = MC_CMD_DRIVE_RIGHT;
						}
					}
				} else if (right_line_correction_active) {
					right_line_correction_active = false;
					tm_printf((UB*)"[STEERING] Brief right pivot did not reacquire the line; switching to left recovery.\r\n");
					line_lost_start_time = now;
					er = vehicle_controller_line_motion_post(MC_CMD_STEER_LEFT, right_line || center_line);
					if (er == E_OK) {
						last_line_motion_command = MC_CMD_STEER_LEFT;
					}
				} else if ((UW)(now - line_lost_start_time) >=
					   LINE_RECOVERY_ARC_MS &&
					   last_line_motion_command != MC_CMD_DRIVE_LEFT) {
					tm_printf((UB*)"[STEERING] Line not found; escalating to sharper left pivot.\r\n");
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
		tk_dly_tsk(1);
	}
}