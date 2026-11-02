/*
 *	subsystem3.h
 *	Subsystem 3 -- Consumer, IR Line Sensors, & Barcode Reader.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define S3_PRI_CONSUMER     16  /* priority band 16-17[cite: 2] */
#define S3_BATCH_SIZE       5   /* report to subsystem 4 every N samples[cite: 2] */

#define THRESHOLD           1500  // Adjusted line-tracking threshold
#define BARCODE_PIN         17    // GP17 digital barcode sensor
#define BARCODE_TIMEOUT_MS  10000  // 10 seconds overall scan timeout
#define QUIET_ZONE_TIMEOUT_MS 3000  // 3s continuous white confirms end of barcode
#define BARCODE_BAR_COUNT   15    // Black bars in each A-D barcode
#define BARCODE_MIN_BAR_WIDTH_US 100
#define BARCODE_EDGE_DEBOUNCE_US 500

#define ADC_CHANNEL_RIGHT   0     // GP26 (sensor mounted right of center)
#define ADC_CHANNEL_CENTER  1     // GP27 (IR_LINE_C_ADC)

typedef enum {
    STATE_FOLLOWING_LINE,
    STATE_READING_BARCODE,
    STATE_LOST_LINE,
    STATE_TIMEOUT_RECOVERY
} RobotState;

typedef enum {
    CMD_UNKNOWN,
    CMD_TURN_LEFT,      // Barcode A
    CMD_TURN_RIGHT,     // Barcode B
    CMD_GO_STRAIGHT,    // Barcode C
    CMD_U_TURN          // Barcode D
} RobotCommand;

/* IF-01: mailbox handle */
IMPORT ID   mbx_samples;

/* IF-04: statistics[cite: 2] */
typedef struct {
    UW  received;
    INT min;
    INT max;
    W   sum;
} APP_STATS;

IMPORT ID       mtx_stats;
IMPORT APP_STATS    app_stats;

/* Private task declaration */
IMPORT void consumer_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM3_H */