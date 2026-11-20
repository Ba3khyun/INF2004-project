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

/* The encoder-core ISR publishes timestamped edges. The consumer owns
 * the partial scan, including reset and decoding, on its own core. */
#define EDGE_QUEUE_SIZE 128u
typedef struct {
	UW us;
	bool high;
} barcode_edge_t;

static barcode_edge_t edges[EDGE_QUEUE_SIZE];
static UW edge_head, edge_tail, dropped_edges;
static UW high_bar_widths[BARCODE_BAR_COUNT], low_bar_widths[BARCODE_BAR_COUNT];
static UW high_bar_count, low_bar_count, last_edge_time, first_edge_time;
static bool scanning, barcode_level_high;

static void barcode_gpio_isr(UINT gpio, UINT events)
{
	UW head = __atomic_load_n(&edge_head, __ATOMIC_RELAXED);
	UW tail = __atomic_load_n(&edge_tail, __ATOMIC_ACQUIRE);
	(void)events; /* The shared IRQ dispatches only this pin's edges. */
	if (head - tail >= EDGE_QUEUE_SIZE) {
		UW lost = __atomic_load_n(&dropped_edges, __ATOMIC_RELAXED);
		__atomic_store_n(&dropped_edges, lost + 1u, __ATOMIC_RELEASE);
		return;
	}
	edges[head % EDGE_QUEUE_SIZE] = (barcode_edge_t){
		in_w(TIMER_TIMERAWL), gpio_get_val(gpio) != 0u
	};
	__atomic_store_n(&edge_head, head + 1u, __ATOMIC_RELEASE);
}

static char decode_barcode(const UW *bar_widths, UW *pattern_out)
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

static void finish_barcode(void)
{
	UW pattern;
	char decoded = '?';
	if (!barcode_level_high && high_bar_count == BARCODE_BAR_COUNT)
		decoded = decode_barcode(high_bar_widths, &pattern);
	if (barcode_level_high && low_bar_count == BARCODE_BAR_COUNT)
		decoded = decode_barcode(low_bar_widths, &pattern);
	if (decoded != '?')
		tm_printf((UB*)"-> Decoded Character: %c\r\n", decoded);
	else if (high_bar_count >= BARCODE_BAR_COUNT || low_bar_count >= BARCODE_BAR_COUNT)
		tm_printf((UB*)"Unknown barcode pattern.\r\n");
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

static void poll_barcode(void)
{
	static UW previous_drops;
	UW tail = __atomic_load_n(&edge_tail, __ATOMIC_RELAXED);
	UW now = in_w(TIMER_TIMERAWL);
	UW head = __atomic_load_n(&edge_head, __ATOMIC_ACQUIRE);
	UW drops = __atomic_load_n(&dropped_edges, __ATOMIC_ACQUIRE);
	if (drops != previous_drops) {
		previous_drops = drops;
		scanning = false;
		__atomic_store_n(&edge_tail, head, __ATOMIC_RELEASE);
		return;
	}
	if (tail == head) {
		barcode_tick(now);
		return;
	}
	while (tail != head) {
		barcode_edge_t edge = edges[tail % EDGE_QUEUE_SIZE];
		__atomic_store_n(&edge_tail, ++tail, __ATOMIC_RELEASE);
		consume_barcode_edge(edge);
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
	UW line_lost_start_time = 0;
#if APP_DEBUG_S3_SENSOR
	UW last_debug_print = current_time.lo;
#endif
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
		
#if APP_DEBUG_S3_SENSOR
		int barcode_val_print = gpio_get_val(BARCODE_PIN) != 0 ? 1 : 0;

		/* Periodic raw ADC/barcode readout: optional during integration. */
		if ((UW)(now - last_debug_print) >= 1000) {
			tm_printf((UB*)"RAW -> R: %4u [%s] | C: %4u [%s] | Barcode (GP28): %d | Threshold: %d\r\n",
				      right, right_line ? "BLACK" : "WHITE",
				      center, center_line ? "BLACK" : "WHITE",
				      barcode_val_print, THRESHOLD);
			last_debug_print = now;
		}
#endif

		poll_barcode();

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
		tk_dly_tsk(1);
	}
}