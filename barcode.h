/*
 * barcode.h
 * Barcode reader on MH-sensor #3, the right-most MH-sensor (Buddy 3).
 * Reads Code 39 barcodes: thick/thin black bars and thick/thin white gaps,
 * framed by '*' start and stop characters (e.g. "*L*").
 *
 * The sensor's ANALOG output (AO) is sampled by the ADC at 2 kHz. Software
 * thresholds with hysteresis turn the voltage into black/white edges, which
 * are timed exactly as before.
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

// Pin from the Design Review pin table (Section 2).
#define BARCODE_AO_PIN         28     // BARCODE_IR_AO: MH-sensor #3 AO (Grove 7)
#define BARCODE_ADC_INPUT      2      // GP28 = ADC input 2

// 1 = AO voltage is HIGHER over a black bar (usual for MH-sensors).
// Set to 0 if the serial output shows the opposite.
#define BARCODE_BAR_IS_HIGH    1

#define BARCODE_SAMPLE_US      500    // ADC sample period: 2 kHz (>= 1 kHz)
#define BARCODE_MIN_CONTRAST   250    // ADC counts (~0.2 V) black above white
#define BARCODE_HYST_PCT       10     // hysteresis either side of the midpoint

#define BARCODE_GLITCH_US      1000   // pulses shorter than this are noise
#define BARCODE_END_GAP_MS     1000    // no edge for this long = pass finished
#define BARCODE_MAX_ELEMENTS   129    // up to 11 characters incl. * and *
#define BARCODE_MAX_TEXT       12

typedef enum
{
    BARCODE_CMD_NONE = 0,
    BARCODE_CMD_LEFT,
    BARCODE_CMD_RIGHT
} barcode_cmd_t;

// Everything measured and decoded in one pass over a barcode.
typedef struct
{
    int           num_elements;                      // bars + gaps
    uint32_t      width_us[BARCODE_MAX_ELEMENTS];    // in the order scanned
    bool          is_thick[BARCODE_MAX_ELEMENTS];    // thick or thin
    bool          valid;                             // decoded OK
    bool          reversed;                          // scanned backwards
    char          text[BARCODE_MAX_TEXT + 1];        // data between the *s
    barcode_cmd_t command;
    int           glitches;                          // pulses < 2 ms removed
    const char *  error;                             // why it failed
} barcode_result_t;

// Starts the ADC sampling. The sensor should be over white floor at this
// moment, so it can learn the floor level.
void        barcode_init(void);

// Call often from the main loop. Returns true when a pass has finished
// and fills *result.
bool        barcode_poll(barcode_result_t *result);

const char *barcode_cmd_name(barcode_cmd_t cmd);

// Latest AO reading and the learned floor (white) and bar (black) levels,
// in ADC counts (0..4095), for tuning and the serial printout.
void        barcode_get_levels(uint16_t *raw, uint16_t *white, uint16_t *black);

#endif // BARCODE_H
