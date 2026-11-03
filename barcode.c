/*
 * barcode.c
 * Reads Code 39 barcodes with an MH-sensor's ANALOG output (AO).
 *
 * A 2 kHz repeating timer reads the ADC (GP27). The reading is compared with
 * two thresholds (hysteresis) set between the learned floor (white) level and
 * the learned bar (black) level, so every black <-> white change becomes a
 * timestamped edge. The main loop turns the edges into widths: bar, gap,
 * bar, ... and, once no edge has arrived for BARCODE_END_GAP_MS, decodes.
 *
 * Code 39: every character is 9 elements (5 bars, 4 gaps), 3 of them thick,
 * with a thin white gap between characters. Normally 2 of the 5 bars and 1 of
 * the 4 gaps are thick, so bars and gaps are sorted separately. That keeps
 * the read correct even if the analog thresholds make bars look slightly
 * wider than gaps. Thick/thin is decided by comparing widths, so the result
 * does not depend on how fast the car moves.
 */

#include "barcode.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define SHOW_BC_LEVELS  1           // print the barcode AO levels every second
#define QUEUE_SIZE        64
#define CHAR_ELEMENTS     9
#define THICK_PER_CHAR    3
#define MIN_RATIO_X10     15        // thick must be >= 1.5 x thin
#define ADC_MAX           4095
#define WHITE_LEARN_SAMPLES 64      // floor readings averaged at start-up

// ---------------------------------------------------------------------------
// Code 39 table: 9 elements, first element = left-most bit, 1 = thick
// ---------------------------------------------------------------------------
typedef struct
{
    char     symbol;
    uint16_t pattern;
} code39_t;

static const code39_t code39[] =
{
    {'0', 0x034}, {'1', 0x121}, {'2', 0x061}, {'3', 0x160}, {'4', 0x031},
    {'5', 0x130}, {'6', 0x070}, {'7', 0x025}, {'8', 0x124}, {'9', 0x064},
    {'A', 0x109}, {'B', 0x049}, {'C', 0x148}, {'D', 0x019}, {'E', 0x118},
    {'F', 0x058}, {'G', 0x00D}, {'H', 0x10C}, {'I', 0x04C}, {'J', 0x01C},
    {'K', 0x103}, {'L', 0x043}, {'M', 0x142}, {'N', 0x013}, {'O', 0x112},
    {'P', 0x052}, {'Q', 0x007}, {'R', 0x106}, {'S', 0x046}, {'T', 0x016},
    {'U', 0x181}, {'V', 0x0C1}, {'W', 0x1C0}, {'X', 0x091}, {'Y', 0x190},
    {'Z', 0x0D0}, {'-', 0x085}, {'.', 0x184}, {' ', 0x0C4}, {'$', 0x0A8},
    {'/', 0x0A2}, {'+', 0x08A}, {'%', 0x02A}, {'*', 0x094}
};

#define CODE39_COUNT  (sizeof(code39) / sizeof(code39[0]))

// ---------------------------------------------------------------------------
// Sampler (timer interrupt) -> main loop queue
// ---------------------------------------------------------------------------
static volatile uint32_t edge_time_us[QUEUE_SIZE];
static volatile uint8_t  edge_level[QUEUE_SIZE];   // 1 = now black, 0 = now white
static volatile uint32_t head = 0;
static volatile uint32_t tail = 0;

// Analog levels in ADC counts x256 (fixed point, so slow averages still move)
static volatile uint32_t white_q = 0;      // floor level
static volatile uint32_t black_q = 0;      // bar level (peak, slowly decays)
static volatile uint16_t raw_now = 0;      // latest reading
static volatile bool     is_black = false;

static struct repeating_timer sample_timer;

// Pass in progress (main loop only)
static uint32_t widths[BARCODE_MAX_ELEMENTS];
static int      num_widths = 0;
static bool     in_pass = false;
static bool     last_black = false;
static uint32_t last_edge_us = 0;
static int      glitches = 0;
static bool     too_long = false;

static uint16_t read_ao(void)
{
    adc_select_input(BARCODE_ADC_INPUT);
    uint16_t v = (uint16_t)adc_read();
    // Work with "higher = blacker" no matter how the module is wired.
    return BARCODE_BAR_IS_HIGH ? v : (uint16_t)(ADC_MAX - v);
}

static void push_edge(uint32_t t, uint8_t black)
{
    uint32_t next = (head + 1) % QUEUE_SIZE;
    if (next == tail) {
        return;                             // queue full: drop the edge
    }
    edge_time_us[head] = t;
    edge_level[head]   = black;
    head = next;
}

// Runs every BARCODE_SAMPLE_US in timer-interrupt context.
static bool barcode_sample_cb(struct repeating_timer *rt)
{
    (void)rt;
    uint32_t t = time_us_32();
    uint16_t v = read_ao();
    raw_now = v;

    uint32_t white = white_q >> 8;

    // Bar level: jump up to any higher reading, then decay slowly (~8 s)
    // back toward the floor, so it follows the paper and the lighting.
    if (((uint32_t)v << 8) > black_q) {
        black_q = (uint32_t)v << 8;
    } else if (black_q > white_q) {
        black_q -= (black_q - white_q) >> 14;
    }

    uint32_t black = black_q >> 8;
    uint32_t span  = (black > white) ? (black - white) : 0;
    if (span < BARCODE_MIN_CONTRAST) {
        span = BARCODE_MIN_CONTRAST;        // ignore small noise on plain floor
    }

    uint32_t mid  = white + span / 2;
    uint32_t hyst = (span * BARCODE_HYST_PCT) / 100;

    if (!is_black && v > mid + hyst) {
        is_black = true;
        push_edge(t, 1);                    // white -> black
    } else if (is_black && v < mid - hyst) {
        is_black = false;
        push_edge(t, 0);                    // black -> white
    }

    // Floor level: slow average (~250 ms) of readings while over white.
    if (!is_black) {
        white_q += (int32_t)(((int32_t)v << 8) - (int32_t)white_q) >> 9;
    }
    return true;                            // keep the timer running
}

// ---------------------------------------------------------------------------
// Decoding
// ---------------------------------------------------------------------------

// Marks the 'count' widest of the elements at w[first], w[first+step], ...
// as thick. Returns false if thick and thin are not clearly different.
static bool mark_widest(const uint32_t *w, int first, int step, int n,
                        int count, bool *thick)
{
    uint32_t thinnest_thick = UINT32_MAX;
    uint32_t widest_thin = 0;

    for (int k = 0; k < count; k++) {
        int best = -1;
        for (int i = first; i < first + n * step; i += step) {
            if (!thick[i] && (best < 0 || w[i] > w[best])) {
                best = i;
            }
        }
        thick[best] = true;
    }

    for (int i = first; i < first + n * step; i += step) {
        if (thick[i]) {
            if (w[i] < thinnest_thick) thinnest_thick = w[i];
        } else if (w[i] > widest_thin) {
            widest_thin = w[i];
        }
    }

    // Thick and thin must be clearly different, or the read is unreliable.
    return (uint64_t)thinnest_thick * 10 >= (uint64_t)widest_thin * MIN_RATIO_X10;
}

static char lookup(const bool *thick)
{
    uint16_t pattern = 0;

    for (int i = 0; i < CHAR_ELEMENTS; i++) {
        if (thick[i]) {
            pattern |= (uint16_t)(1u << (CHAR_ELEMENTS - 1 - i));
        }
    }
    for (unsigned i = 0; i < CODE39_COUNT; i++) {
        if (code39[i].pattern == pattern) {
            return code39[i].symbol;
        }
    }
    return 0;
}

// Decodes one 9-element character and fills thick[0..8].
// 1st try: 2 thick of the 5 bars + 1 thick of the 4 gaps (most characters).
// 2nd try: 3 thick of all 9 (needed for $ / + %, which have 3 thick gaps).
static char decode_char(const uint32_t *w, bool *thick)
{
    char c;

    for (int i = 0; i < CHAR_ELEMENTS; i++) thick[i] = false;
    if (mark_widest(w, 0, 2, 5, 2, thick) &&        // bars: 0,2,4,6,8
        mark_widest(w, 1, 2, 4, 1, thick)) {        // gaps: 1,3,5,7
        c = lookup(thick);
        if (c != 0) {
            return c;
        }
    }

    for (int i = 0; i < CHAR_ELEMENTS; i++) thick[i] = false;
    if (mark_widest(w, 0, 1, CHAR_ELEMENTS, THICK_PER_CHAR, thick)) {
        return lookup(thick);
    }
    return 0;
}

// Decodes elements in the given order. Fills text and thick[].
static bool decode_symbol(const uint32_t *w, int n, char *text, bool *thick,
                          const char **error)
{
    if (n < 29 || ((n + 1) % 10) != 0) {
        *error = "wrong number of bars/gaps (Code 39 needs 29, 39, 49 ...)";
        return false;
    }

    int  chars = (n + 1) / 10;
    char decoded[BARCODE_MAX_TEXT + 2];

    for (int c = 0; c < chars; c++) {
        decoded[c] = decode_char(&w[c * 10], &thick[c * 10]);
        if (decoded[c] == 0) {
            *error = "a character did not match the Code 39 table";
            return false;
        }
        if (c < chars - 1) {
            thick[c * 10 + 9] = false;      // gap between characters
        }
    }

    if (decoded[0] != '*' || decoded[chars - 1] != '*') {
        *error = "missing * start or stop character";
        return false;
    }

    int len = chars - 2;
    memcpy(text, &decoded[1], (size_t)len);
    text[len] = '\0';
    return true;
}

static barcode_cmd_t command_for(const char *text)
{
    // First data character decides the command. Change to suit the course.
    switch (text[0]) {
        case 'L': case 'A': return BARCODE_CMD_LEFT;
        case 'R': case 'B': return BARCODE_CMD_RIGHT;
        default:            return BARCODE_CMD_NONE;
    }
}

// Rough thick/thin split, only used for display when decoding fails.
static void mark_thick_by_threshold(barcode_result_t *r)
{
    uint32_t lo = UINT32_MAX, hi = 0;
    for (int i = 0; i < r->num_elements; i++) {
        if (r->width_us[i] < lo) lo = r->width_us[i];
        if (r->width_us[i] > hi) hi = r->width_us[i];
    }
    uint32_t mid = (lo + hi) / 2;
    for (int i = 0; i < r->num_elements; i++) {
        r->is_thick[i] = r->width_us[i] > mid;
    }
}

static void finish_pass(barcode_result_t *r)
{
    memset(r, 0, sizeof(*r));
    r->num_elements = num_widths;
    r->glitches     = glitches;
    memcpy(r->width_us, widths, sizeof(uint32_t) * (size_t)num_widths);

    if (too_long) {
        r->error = "too many bars/gaps for the buffer";
    } else if (decode_symbol(widths, num_widths, r->text, r->is_thick,
                             &r->error)) {
        r->valid = true;
    } else {
        // Try backwards (car moving the other way).
        static uint32_t rev[BARCODE_MAX_ELEMENTS];       // static: keeps ~650 B
        static bool     rev_thick[BARCODE_MAX_ELEMENTS]; // off the stack
        const char *rev_error = NULL;

        for (int i = 0; i < num_widths; i++) {
            rev[i] = widths[num_widths - 1 - i];
        }
        if (decode_symbol(rev, num_widths, r->text, rev_thick, &rev_error)) {
            r->valid    = true;
            r->reversed = true;
            r->error    = NULL;
            for (int i = 0; i < num_widths; i++) {
                r->is_thick[i] = rev_thick[num_widths - 1 - i];
            }
        }
    }

    if (r->valid) {
        r->command = command_for(r->text);
    } else {
        mark_thick_by_threshold(r);
    }

    in_pass    = false;
    num_widths = 0;
}

// ---------------------------------------------------------------------------
// Public functions
// ---------------------------------------------------------------------------

void barcode_init(void)
{
    adc_init();
    adc_gpio_init(BARCODE_AO_PIN);          // GP28 as an analog input

    // Learn the floor level: the sensor should be over white right now.
    uint32_t sum = 0;
    for (int i = 0; i < WHITE_LEARN_SAMPLES; i++) {
        sum += read_ao();
        sleep_us(BARCODE_SAMPLE_US);
    }
    white_q  = (sum / WHITE_LEARN_SAMPLES) << 8;
    black_q  = white_q + ((uint32_t)BARCODE_MIN_CONTRAST << 8);
    is_black = false;

    // Negative period = fixed start-to-start spacing (2 kHz).
    add_repeating_timer_us(-BARCODE_SAMPLE_US, barcode_sample_cb, NULL,
                           &sample_timer);
}

bool barcode_poll(barcode_result_t *result)
{
    while (tail != head) {
        uint32_t t     = edge_time_us[tail];
        bool     black = (edge_level[tail] != 0);
        tail = (tail + 1) % QUEUE_SIZE;

        if (!in_pass) {
            if (black) {                    // first bar starts a pass
                in_pass      = true;
                num_widths   = 0;
                glitches     = 0;
                too_long     = false;
                last_black   = true;
                last_edge_us = t;
            }
            continue;
        }

        if (black == last_black) {
            continue;                       // repeated level, ignore
        }

        uint32_t dt = t - last_edge_us;

        if (dt < BARCODE_GLITCH_US) {
            // Noise: undo the previous edge and skip this one.
            glitches++;
            if (num_widths > 0) {
                num_widths--;
                last_edge_us -= widths[num_widths];
                last_black    = black;
            } else {
                in_pass = false;
            }
            continue;
        }

        if (num_widths < BARCODE_MAX_ELEMENTS) {
            widths[num_widths++] = dt;
        } else {
            too_long = true;
        }
        last_edge_us = t;
        last_black   = black;
    }

    if (in_pass && (time_us_32() - last_edge_us) > BARCODE_END_GAP_MS * 1000u) {
        finish_pass(result);
        return true;
    }
    return false;
}

const char *barcode_cmd_name(barcode_cmd_t cmd)
{
    switch (cmd) {
        case BARCODE_CMD_LEFT:  return "TURN LEFT";
        case BARCODE_CMD_RIGHT: return "TURN RIGHT";
        default:                return "no command for this code";
    }
}

void barcode_get_levels(uint16_t *raw, uint16_t *white, uint16_t *black)
{
    *raw   = raw_now;
    *white = (uint16_t)(white_q >> 8);
    *black = (uint16_t)(black_q >> 8);
}
