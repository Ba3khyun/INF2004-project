/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (cyw43_utk_task).  The IS-06 Wi-Fi
 *	Telemetry state machine (app_program/subsystem1/) drives it through the
 *	non-blocking calls in mqtt_utk.h from wifi_app_poll(), which that same
 *	task calls on every poll iteration.  mqtt.c's own keepalive timer rides
 *	the sys_check_timeouts() polling the task already does -- nothing runs
 *	on any other task or core.
 *
 *	Changes from the original heartbeat demo: no self-rearming heartbeat
 *	and no self-retry timer (reconnection and its bounded back-off are
 *	owned by the application state machine), a TCP-progress liveness
 *	counter for <= 2 s link-loss detection, and binary payload pass-through.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

/* Keepalive only matters while idle: telemetry is published every 100 ms,
   and loss is detected by the application from TCP progress (1.5 s). */
#define MQTT_UTK_KEEPALIVE_S	10

/* Inbound message staging.  A publish arrives as one topic callback
   followed by one or more data callbacks.  Commands are 3 bytes; anything
   longer is cut here and rejected by the application as a bad length. */
#define MQTT_UTK_RX_TOPIC_MAX	48
#define MQTT_UTK_RX_DATA_MAX	16

static mqtt_client_t	*client;	/* allocated once, never freed */
static ip_addr_t	broker_addr;
static u8_t		state = MQTT_UTK_ST_IDLE;
static u32_t		unacked;
static u32_t		last_progress_ms;
static mqtt_utk_rx_cb_t	rx_cb;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static u8_t		rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t		rx_len;
static u32_t		rx_tot_len;

void mqtt_utk_set_rx_callback(mqtt_utk_rx_cb_t cb)
{
	rx_cb = cb;
}

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;
	for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';
	rx_len = 0;
	rx_tot_len = tot_len;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;
	for(i = 0; i < len; i++) {
		if(rx_len < MQTT_UTK_RX_DATA_MAX) {
			rx_data[rx_len++] = data[i];
		}
	}
	if((flags & MQTT_DATA_FLAG_LAST) != 0 && rx_cb != NULL) {
		rx_cb(rx_topic, rx_data, rx_len, rx_tot_len);
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		if(state == MQTT_UTK_ST_CONNECTED) {
			state = MQTT_UTK_ST_SUBSCRIBED;
		}
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
		state = MQTT_UTK_ST_FAILED;
	}
}

/* QoS 0: mqtt.c completes the request when TCP acknowledges the data
 * (mqtt_tcp_sent_cb), so an ERR_OK here is end-to-end transport progress. */
static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(unacked > 0) {
		unacked--;
	}
	if(result == ERR_OK) {
		last_progress_ms = sys_now();
	}
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		state = MQTT_UTK_ST_CONNECTED;
		last_progress_ms = sys_now();
		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");
	} else {
		/* refused, TCP error, keepalive timeout or broker disconnect */
		state = MQTT_UTK_ST_FAILED;
		tm_printf((UB*)"mqtt: connection status %d\n", (int)status);
	}
}

int32_t mqtt_utk_connect(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(state == MQTT_UTK_ST_CONNECTING || state == MQTT_UTK_ST_CONNECTED ||
	   state == MQTT_UTK_ST_SUBSCRIBED) {
		return MQTT_UTK_OK;
	}
	if(client == NULL) {
		client = mqtt_client_new();	/* lwIP heap, once */
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return MQTT_UTK_E_BUSY;
		}
	}
	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return MQTT_UTK_E_PARAM;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	/* a previous session may still hold its TCP pcb */
	mqtt_disconnect(client);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	/* Callbacks must be registered before subscribing, or the first
	 * delivery has nowhere to go; mqtt_client_connect() keeps them. */
	mqtt_set_inpub_callback(client, incoming_publish_cb, incoming_data_cb, NULL);

	unacked = 0;
	state = MQTT_UTK_ST_CONNECTING;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		state = MQTT_UTK_ST_FAILED;
		return MQTT_UTK_E_CONN;
	}
	return MQTT_UTK_OK;
}

void mqtt_utk_disconnect(void)
{
	if(client != NULL) {
		mqtt_disconnect(client);	/* suppresses connection_cb */
	}
	state = MQTT_UTK_ST_IDLE;
	unacked = 0;
}

int32_t mqtt_utk_subscribe(const char *topic)
{
	err_t	err;

	if(client == NULL || state != MQTT_UTK_ST_CONNECTED) {
		return MQTT_UTK_E_CONN;
	}
	err = mqtt_subscribe(client, topic, 0 /* qos */, subscribe_done_cb, NULL);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n", (int)err);
		return (err == ERR_MEM)? MQTT_UTK_E_BUSY: MQTT_UTK_E_CONN;
	}
	return MQTT_UTK_OK;
}

int32_t mqtt_utk_publish(const char *topic, const uint8_t *data, uint16_t len)
{
	err_t	err;

	if(client == NULL || (state != MQTT_UTK_ST_SUBSCRIBED &&
			      state != MQTT_UTK_ST_CONNECTED)) {
		return MQTT_UTK_E_CONN;
	}
	err = mqtt_publish(client, topic, data, len, 0 /* qos */, 0 /* retain */,
			   publish_done_cb, NULL);
	if(err == ERR_OK) {
		if(unacked == 0) {
			last_progress_ms = sys_now();	/* start the liveness clock */
		}
		unacked++;
		return MQTT_UTK_OK;
	}
	return (err == ERR_CONN)? MQTT_UTK_E_CONN: MQTT_UTK_E_BUSY;
}

uint32_t mqtt_utk_state(void)
{
	return state;
}

uint32_t mqtt_utk_unacked(void)
{
	return unacked;
}

uint32_t mqtt_utk_last_progress_ms(void)
{
	return last_progress_ms;
}
