#ifndef CYW43_UTK_H
#define CYW43_UTK_H

#include <stdint.h>

typedef struct {
    volatile int32_t init_result;
    volatile uint32_t init_complete;
    volatile uint32_t ready;
    volatile uint32_t scan_active;
    volatile uint32_t scan_complete;
    volatile uint32_t scan_count;
    volatile int32_t strongest_rssi;
    volatile uint32_t poll_count;
    volatile uint32_t owner_prc;
    uint8_t mac[6];
    /* Only meaningful on a NET=lwip build -- see cyw43_utk_task(). link_status
     * mirrors cyw43_tcpip_link_status()'s CYW43_LINK_* values (cyw43.h):
     * negative = down/join failed, 1 = joined, 2 = joined + got an IP. */
    volatile int32_t link_status;
    volatile uint32_t ip_addr;
} T_CYW43_UTK_STATUS;

int32_t cyw43_utk_start(void);
void cyw43_utk_get_status(T_CYW43_UTK_STATUS *status);
void cyw43_utk_task(int32_t stacd, void *exinf);

/* ---------------------------------------------------------------------
 * NET=lwip MQTT=1 only: application control of the association.
 * Call ONLY from code running inside cyw43_utk_task (i.e. wifi_app_poll).
 * --------------------------------------------------------------------- */
#define CYW43_UTK_LINK_DOWN     0   /* not associated */
#define CYW43_UTK_LINK_JOINING  1   /* associating / waiting for DHCP */
#define CYW43_UTK_LINK_UP       2   /* associated with an IP address */
#define CYW43_UTK_LINK_FAIL     3   /* join failed: no SSID, bad auth, ... */

int32_t cyw43_utk_join(void);       /* request a join; 0 = request accepted */
void    cyw43_utk_leave(void);      /* drop the association */
int32_t cyw43_utk_link(void);       /* CYW43_UTK_LINK_* */

/* Provided by the application (IS-06 Buddy 1, app_program/subsystem1/
 * wifi_telemetry.c).  With MQTT=1, cyw43_utk_task calls it once per poll
 * iteration, so all Wi-Fi/MQTT application logic runs in this one task --
 * the only task allowed to touch the cyw43/lwIP/MQTT stack.  It must never
 * block. */
void    wifi_app_poll(void);

#endif
