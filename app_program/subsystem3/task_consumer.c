/*
 *	task_consumer.c
 *	Subsystem 3 -- Consumer task combined with IR sensors, Code 39 decoding, and Subsystem 2 motor control.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"   /* IF-01/02: SAMPLE_MSG[cite: 4] */
#include "../subsystem2/drv_motion.h"   /* Subsystem 2 Motor Control API */
#include "../subsystem4/subsystem4.h"   /* IF-03: mbf_summary[cite: 4] */

static UW edge_intervals[MAX_EDGES];
static UW edge_count = 0;
static UW last_edge_time = 0;
static bool new_barcode_triggered = false;
static bool previous_barcode_is_black = false;

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
	UW barcode_start_time = 0;
	UW last_debug_print = current_time.lo;
	UB last_line_motion_command = 0;
	previous_barcode_is_black = gpio_get_val(BARCODE_PIN) != 0;

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

		bool right_line = (right > THRESHOLD); 
		bool center_line = (center > THRESHOLD);
		
		// Barcode pin: 0 for white (LED on), 1 for black (LED off)
		bool barcode_is_black = gpio_get_val(BARCODE_PIN) != 0;
		if (barcode_is_black != previous_barcode_is_black) {
			if (edge_count == 0) {
				edge_count = 1;
				new_barcode_triggered = true;
			} else if (edge_count < MAX_EDGES) {
				edge_intervals[edge_count - 1] = now - last_edge_time;
				edge_count++;
			}
			last_edge_time = now;
			previous_barcode_is_black = barcode_is_black;
		}
		int barcode_val_print = barcode_is_black ? 1 : 0;

		// Print debug status once per second.
		if ((UW)(now - last_debug_print) >= 1000) {
			tm_printf((UB*)"RAW -> R: %4u [%s] | C: %4u [%s] | Barcode (GP2): %d [%s] | Threshold: %d\r\n",
				  right, right_line ? "BLACK" : "WHITE",
				  center, center_line ? "BLACK" : "WHITE",
				  barcode_val_print, barcode_is_black ? "BLACK (1)" : "WHITE (0)",
				  THRESHOLD);
			last_debug_print = now;
		}

		switch (current_state) {
			case STATE_FOLLOWING_LINE:
				// 1. Check if line is completely lost
				if (!right_line && !center_line) {
					tm_printf((UB*)"[STEERING] Line lost; starting gentle left recovery arc.\r\n");
					er = motion_cmd_post(MC_CMD_STEER_LEFT);
					if (er != E_OK) {
						tm_printf((UB*)"[STEERING] Left recovery command failed: %d\r\n", er);
					} else {
						last_line_motion_command = MC_CMD_STEER_LEFT;
					}
					current_state = STATE_LOST_LINE;
					break;
				}

				// Center detection takes priority so it always commands straight.
				UB line_motion_command;
				if (center_line) {
					tm_printf((UB*)"[STEERING] Center aligned -> Go Straight\r\n");
					line_motion_command = MC_CMD_DRIVE_FORWARD;
				} 
				else if (right_line) {
					tm_printf((UB*)"[STEERING] Line under right sensor -> Gentle right correction\r\n");
					line_motion_command = MC_CMD_STEER_RIGHT;
				} 
				if (line_motion_command != last_line_motion_command ||
				    motion_is_idle()) {
					er = motion_cmd_post(line_motion_command);
					if (er != E_OK) {
						tm_printf((UB*)"[STEERING] Motion command failed: %d\r\n", er);
					} else {
						last_line_motion_command = line_motion_command;
					}
				}

				// 3. Check if barcode scan triggered
				if (new_barcode_triggered) {
					new_barcode_triggered = false;
					edge_count = 1; 
					barcode_start_time = now;
					last_edge_time = barcode_start_time;
					
					tm_printf((UB*)"Transitioning to STATE_READING_BARCODE...\r\n");
					current_state = STATE_READING_BARCODE;
				}
				break;

			case STATE_READING_BARCODE:
			{
				if ((UW)(now - barcode_start_time) > BARCODE_TIMEOUT_MS) {
					tm_printf((UB*)"TIMEOUT: Barcode read took too long! Aborting...\r\n");
					edge_count = 0;
					current_state = STATE_FOLLOWING_LINE; 
					break;
				}

				if (barcode_is_black && (UW)(now - last_edge_time) > QUIET_ZONE_TIMEOUT_MS) {
					tm_printf((UB*)"Barcode scan complete! Total edges captured: %u\r\n", edge_count);

					if (edge_count >= 6) {
						UW total_duration = 0;
						for (UW i = 0; i < edge_count - 1; i++) {
							total_duration += edge_intervals[i];
						}
						UW avg_duration = total_duration / (edge_count - 1);

						UW pattern_signature = 0;
						for (UW i = 0; i < edge_count - 1; i++) {
							pattern_signature <<= 1;
							if (edge_intervals[i] > (avg_duration * 13 / 10)) {
								pattern_signature |= 1; 
							} else {
								pattern_signature |= 0; 
							}
						}

						tm_printf((UB*)">>> CAPTURED PATTERN SIGNATURE: 0x%x <<<\r\n", pattern_signature);

						char decoded_char = '?';
						if (pattern_signature == 0x128)      decoded_char = 'A';
						else if (pattern_signature == 0x144) decoded_char = 'B';
						else if (pattern_signature == 0x182) decoded_char = 'C';
						else if (pattern_signature == 0x10A) decoded_char = 'D';

						RobotCommand cmd = CMD_UNKNOWN;
						switch (decoded_char) {
							case 'A': tm_printf((UB*)"-> Decoded 'A': Turn Left\r\n"); cmd = CMD_TURN_LEFT; break;
							case 'B': tm_printf((UB*)"-> Decoded 'B': Turn Right\r\n"); cmd = CMD_TURN_RIGHT; break;
							case 'C': tm_printf((UB*)"-> Decoded 'C': Go Straight\r\n"); cmd = CMD_GO_STRAIGHT; break;
							case 'D': tm_printf((UB*)"-> Decoded 'D': U-Turn\r\n"); cmd = CMD_U_TURN; break;
							default: tm_printf((UB*)"-> Unknown pattern (0x%x). Resuming line tracking.\r\n", pattern_signature); break;
						}

						UB motion_command = 0;
						switch (cmd) {
							case CMD_TURN_LEFT:  motion_command = MC_CMD_LEFT; break;
							case CMD_TURN_RIGHT: motion_command = MC_CMD_RIGHT; break;
							case CMD_GO_STRAIGHT: motion_command = MC_CMD_FORWARD; break;
							case CMD_U_TURN:
								tm_printf((UB*)"[BARCODE] U-turn is not supported by Subsystem 2.\r\n");
								break;
							default:
								break;
						}
						if (motion_command != 0) {
							er = motion_cmd_post(motion_command);
							if (er != E_OK) {
								tm_printf((UB*)"[BARCODE] Motion command failed: %d\r\n", er);
							}
						}
					} else {
						tm_printf((UB*)"-> Scan rejected: Edge count too low (%u)\r\n", edge_count);
					}

					edge_count = 0;
					current_state = STATE_FOLLOWING_LINE;
				}
				break;
			}

			case STATE_LOST_LINE:
				if (right_line || center_line) {
					tm_printf((UB*)"[STEERING] Line reacquired; resuming line following.\r\n");
					last_line_motion_command = 0;
					current_state = STATE_FOLLOWING_LINE;
				} else if (last_line_motion_command != MC_CMD_STEER_LEFT) {
					er = motion_cmd_post(MC_CMD_STEER_LEFT);
					if (er != E_OK) {
						tm_printf((UB*)"[STEERING] Left line recovery command failed: %d\r\n", er);
					} else {
						last_line_motion_command = MC_CMD_STEER_LEFT;
					}
				}
				break;

			case STATE_TIMEOUT_RECOVERY:
				current_state = STATE_FOLLOWING_LINE;
				break;
		}

		tk_dly_tsk(1);
	}
}