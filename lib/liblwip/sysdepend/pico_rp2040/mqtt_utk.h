/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Every function here MUST be called from the single task that owns the
 *	cyw43/lwIP stack (cyw43_utk_task, which also runs the IS-06 Wi-Fi
 *	Telemetry Task logic through wifi_app_poll()).  None of them blocks:
 *	results arrive later through mqtt_utk_state() and the RX callback.
 *
 *	Plain stdint types only, so the header can be included from both the
 *	kernel namespace (tk/tkernel.h) and the pico-sdk namespace.
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

#include <stdint.h>

/* mqtt_utk_state() */
#define MQTT_UTK_ST_IDLE	0u
#define MQTT_UTK_ST_CONNECTING	1u
#define MQTT_UTK_ST_CONNECTED	2u	/* CONNACK received */
#define MQTT_UTK_ST_SUBSCRIBED	3u	/* SUBACK received */
#define MQTT_UTK_ST_FAILED	4u	/* refused, closed or timed out */

/* return codes */
#define MQTT_UTK_OK		0
#define MQTT_UTK_E_BUSY		(-1)	/* output ring / request slots full */
#define MQTT_UTK_E_CONN		(-2)	/* not connected */
#define MQTT_UTK_E_PARAM	(-3)

/* Called once per complete inbound PUBLISH (same task).  data holds the
   first len bytes (len <= MQTT_UTK_RX_DATA_MAX); tot_len is the full
   payload length as announced by the broker. */
typedef void (*mqtt_utk_rx_cb_t)(const char *topic, const uint8_t *data,
				 uint16_t len, uint32_t tot_len);

void	 mqtt_utk_set_rx_callback(mqtt_utk_rx_cb_t cb);
int32_t	 mqtt_utk_connect(void);	/* start; broker from wifi_secrets.h */
void	 mqtt_utk_disconnect(void);	/* abort the session, no callback */
int32_t	 mqtt_utk_subscribe(const char *topic);	/* QoS 0 */
int32_t	 mqtt_utk_publish(const char *topic, const uint8_t *data, uint16_t len);
uint32_t mqtt_utk_state(void);

/* Liveness: publishes not yet acknowledged by TCP, and sys_now() of the
   last TCP progress (an ack, or the first publish after an idle period). */
uint32_t mqtt_utk_unacked(void);
uint32_t mqtt_utk_last_progress_ms(void);

#endif /* MQTT_UTK_H */
