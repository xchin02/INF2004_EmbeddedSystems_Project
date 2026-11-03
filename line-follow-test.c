/*
 * line_follow.c
 * Buddy 3 line follower for the Robo Pico car.
 *
 * The left and right MH-sensors straddle a thin black line.
 *   - Both see floor  -> the line is between them: drive straight.
 *     moveForward() keeps both wheels on the same encoder count, so
 *     "straight" really is straight and the car only drifts slowly.
 *   - ONE sensor sees black -> either the car has drifted, or it has reached
 *     a + junction at a slight angle (one sensor always touches the crossbar
 *     first). So it keeps driving straight for CONFIRM_MM first:
 *       - other sensor also sees black -> junction (see below)
 *       - still only that sensor        -> real drift: steer back
 *         (LEFT sees line -> drifted right -> steer left, and vice versa)
 *   - A sensor that stays on the line longer than SPIN_AFTER_MS means a
 *     sharp bend or corner, so the car spins on the spot towards that side
 *     until the line is back between the sensors.
 *   - Both see black  -> a + junction: drive straight across on the encoders
 *     for CROSS_MM with the sensors ignored, so the crossbar's edges can't
 *     make it steer, then go back to following the line.
 *
 * Barcodes (Buddy 3): the barcode MH-sensor's analog output (GP28) is sampled
 * in the background by barcode.c. check_barcode() is called every loop, so a
 * barcode is decoded while the car keeps driving; the LEFT/RIGHT command is
 * kept in pending_cmd for the navigation logic to act on.
 *
 * Steering sets the wheel speeds directly (driveWheels), NOT through
 * moveForward(): moveForward()'s encoder sync can correct each wheel by up
 * to 15%, which is more than the steering difference and would cancel it.
 * The line sensors are what close the loop while steering.
 *
 * GP20 button: press to start, press again to stop. While waiting, the raw
 * sensor levels are printed whenever they change, so you can check
 * IR_LINE_IS_HIGH by sliding each sensor over the line.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "ir-line-sensor.h"
#include "barcode.h"
#include "wheel.h"

// ---------- Speeds, % of full power (0..100) ----------
#define STRAIGHT_SPEED 10 // both wheels while the line is between the sensors
#define STEER_OUTER 35    // outside wheel while steering back towards the line
#define STEER_INNER 0     // inside wheel while steering (0 = pivot around it)
#define SPIN_SPEED 35     // each wheel (one forward, one back) when spinning on the spot

// ---------- Timing ----------
#define SPIN_AFTER_MS 300 // a sensor still on the line after this long -> spin on the spot
                          // (lots of "SPIN" in the log on gentle drifts -> raise it)
#define CLEAR_MS 4        // a sensor must read floor this long before it counts as off the line
                          // (stops a ragged line edge from flickering the steering)
#define LOOP_MS 1         // how often the sensors are read

// ---------- Distances, measured with the wheel encoders ----------
// MEASURE THESE TWO: run encoderCheck() and turn one wheel exactly once by hand
// for ENC_COUNTS_PER_REV; measure the wheel across for WHEEL_DIAMETER_MM.
#define ENC_COUNTS_PER_REV 1440 // encoder counts for ONE wheel revolution (placeholder!)
#define WHEEL_DIAMETER_MM 65    // wheel diameter in mm (placeholder!)
#define MM_TO_COUNTS(mm) ((int32_t)((float)(mm) * (float)ENC_COUNTS_PER_REV / (3.14159f * (float)WHEEL_DIAMETER_MM) + 0.5f))

#define CONFIRM_MM 6     // one sensor sees black: drive straight this far waiting for the other one.
                         // Needs >= sensor spacing x tan(approach angle): 32 mm x tan(10 deg) = 5.6 mm.
                         // Car still steers on entering a junction -> raise it;
                         // reacts too late to drift / loses the line on bends -> lower it.
#define CROSS_MM 35      // junction: drive straight at least this far with the sensors ignored
                         // (crossbar width + a margin). Reacts to the crossbar's far edge -> raise it.
#define CROSS_MAX_MM 100 // junction: never drive blind further than this

#define START_BUTTON_PIN 20 // Robo Pico user button (reads LOW when pressed)
#define LOG_CHANGES 1       // 1 = print each steering change over USB (handy while tuning)

typedef enum
{
    ACT_STOP,
    ACT_STRAIGHT,
    ACT_STEER_LEFT,
    ACT_SPIN_LEFT,
    ACT_STEER_RIGHT,
    ACT_SPIN_RIGHT,
} action_t;

typedef enum
{
    MODE_FOLLOW,   // normal line following
    MODE_CONFIRM,  // one sensor saw black: drift or the start of a junction?
    MODE_CROSSING, // crossing a junction straight, sensors ignored
} follow_mode_t;

static const char *action_name(action_t a)
{
    switch (a)
    {
    case ACT_STRAIGHT:
        return "straight";
    case ACT_STEER_LEFT:
        return "steer LEFT  (drifted right)";
    case ACT_SPIN_LEFT:
        return "SPIN LEFT   (sharp bend)";
    case ACT_STEER_RIGHT:
        return "steer RIGHT (drifted left)";
    case ACT_SPIN_RIGHT:
        return "SPIN RIGHT  (sharp bend)";
    default:
        return "stop";
    }
}

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

// ---------- Distance travelled, from the encoders ----------
typedef struct
{
    int32_t left, right;
} enc_mark_t;

static enc_mark_t enc_mark(void)
{
    enc_mark_t m = {getLeftCount(), getRightCount()};
    return m;
}

// Average distance both wheels have moved since the mark, in encoder counts.
// abs() so it works whichever way round the encoders count.
static int32_t dist_since(enc_mark_t m)
{
    int32_t dl = (int32_t)((uint32_t)getLeftCount() - (uint32_t)m.left);
    int32_t dr = (int32_t)((uint32_t)getRightCount() - (uint32_t)m.right);
    return (abs(dl) + abs(dr)) / 2;
}

// Forward with both wheels at this speed (%). Goes through moveForward(),
// so the encoders keep the car on its current heading.
static void drive(int left, int right)
{
    moveForward(left - BASE_SPEED, right - BASE_SPEED);
}

static void apply(action_t a)
{
    switch (a)
    {
    case ACT_STRAIGHT:
        drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
        break;
    case ACT_STEER_LEFT:
        driveWheels(STEER_INNER, STEER_OUTER); // left wheel slower; no encoder sync (see top)
        break;
    case ACT_STEER_RIGHT:
        driveWheels(STEER_OUTER, STEER_INNER); // right wheel slower
        break;
    case ACT_SPIN_LEFT:
        driveWheels(-SPIN_SPEED, +SPIN_SPEED);
        break;
    case ACT_SPIN_RIGHT:
        driveWheels(+SPIN_SPEED, -SPIN_SPEED);
        break;
    default:
        stopMotors();
        break;
    }
}

// Steer towards `side` (-1 left, +1 right); spin once it has gone on too long
static action_t steer_action(int side, uint32_t steering_ms)
{
    bool spin = steering_ms >= SPIN_AFTER_MS;
    if (side < 0)
        return spin ? ACT_SPIN_LEFT : ACT_STEER_LEFT;
    return spin ? ACT_SPIN_RIGHT : ACT_STEER_RIGHT;
}

// ---------- Barcode (Buddy 3) ----------
static barcode_cmd_t pending_cmd = BARCODE_CMD_NONE; // last LEFT/RIGHT read, not used yet

// Call every loop. When a pass over a barcode has finished, prints the result
// and keeps a valid LEFT/RIGHT command in pending_cmd.
static void check_barcode(void)
{
    static barcode_result_t result; // static: ~700 B, too big for the stack

    if (!barcode_poll(&result))
        return;

    unsigned long t = (unsigned long)now_ms();
    if (!result.valid)
    {
        printf("[%7lu ms] barcode not decoded: %s\n", t, result.error);
    }
    else if (result.command == BARCODE_CMD_NONE)
    {
        printf("[%7lu ms] barcode \"%s\" -> no command for this code\n", t, result.text);
    }
    else
    {
        pending_cmd = result.command;
        printf("[%7lu ms] barcode \"%s\"%s -> %s\n", t, result.text,
               result.reversed ? " (read backwards)" : "", barcode_cmd_name(result.command));
    }
}

static bool button_down(void)
{
    return !gpio_get(START_BUTTON_PIN);
}

// Waits for one full press-and-release of the button. Meanwhile prints the
// raw sensor levels whenever they change, to check IR_LINE_IS_HIGH:
// over the black line each sensor should say "line".
static void wait_for_click(void)
{
    int last_l = -1, last_r = -1;
    while (!button_down())
    {
        int l = ir_line_left_raw();
        int r = ir_line_right_raw();
        if (l != last_l || r != last_r)
        {
            printf("  sensors raw: L=%d (%s)  R=%d (%s)\n",
                   l, ir_line_is_left_on_line() ? "line " : "floor",
                   r, ir_line_is_right_on_line() ? "line " : "floor");
            last_l = l;
            last_r = r;
        }
        check_barcode();
        sleep_ms(10);
    }
    sleep_ms(30); // debounce
    while (button_down())
        sleep_ms(10);
    sleep_ms(30);
}

// Follows the line until the button is pressed
static void follow_line(void)
{
    const int32_t confirm_counts = MM_TO_COUNTS(CONFIRM_MM);
    const int32_t cross_counts = MM_TO_COUNTS(CROSS_MM);
    const int32_t cross_max_counts = MM_TO_COUNTS(CROSS_MAX_MM);

    action_t current = ACT_STOP;
    follow_mode_t mode = MODE_FOLLOW;
    int steer_side = 0;           // -1 steering left, +1 steering right, 0 not steering
    uint32_t steer_start = 0;     // when the current steer began
    int confirm_side = 0;         // MODE_CONFIRM: -1 left sensor saw black, +1 right sensor
    enc_mark_t mark = enc_mark(); // where MODE_CONFIRM / MODE_CROSSING started
    uint32_t junctions = 0;

    uint32_t start = now_ms();
    uint32_t left_seen = start - CLEAR_MS; // last time each sensor saw the line
    uint32_t right_seen = start - CLEAR_MS;

    while (!button_down())
    {
        uint32_t now = now_ms();

        // On the line as soon as a sensor sees it; off only after CLEAR_MS of floor
        if (ir_line_is_left_on_line())
            left_seen = now;
        if (ir_line_is_right_on_line())
            right_seen = now;
        bool left = (now - left_seen) < CLEAR_MS;
        bool right = (now - right_seen) < CLEAR_MS;

        action_t want = ACT_STRAIGHT;

        // Both sensors on black (outside a crossing) = a junction: cross it straight
        if (mode != MODE_CROSSING && left && right)
        {
            mode = MODE_CROSSING;
            mark = enc_mark();
            steer_side = 0;
            junctions++;
            if (LOG_CHANGES)
                printf("[%7lu ms] junction #%lu: crossing straight\n",
                       (unsigned long)now, (unsigned long)junctions);
        }

        switch (mode)
        {
        case MODE_CROSSING:
        {
            // Straight on the encoders, sensors ignored. Leave once past the
            // crossbar (CROSS_MM) and the sensors aren't both on black any more.
            // A single sensor still on black then is the main line's edge, so
            // normal following takes over and corrects it. CROSS_MAX_MM is the
            // safety cap (e.g. a wide black patch).
            int32_t d = dist_since(mark);
            if ((d >= cross_counts && !(left && right)) || d >= cross_max_counts)
            {
                mode = MODE_FOLLOW;
                if (LOG_CHANGES)
                    printf("[%7lu ms] past junction, following the line\n", (unsigned long)now);
            }
            break; // want stays ACT_STRAIGHT
        }

        case MODE_CONFIRM:
        {
            bool first = (confirm_side < 0) ? left : right;
            if (!first)
            {
                // That sensor is back on floor (or the other one took over):
                // it was a brief touch, carry on following
                mode = MODE_FOLLOW;
            }
            else if (dist_since(mark) >= confirm_counts)
            {
                // Still only one sensor on black after CONFIRM_MM: a real drift
                mode = MODE_FOLLOW;
                steer_side = confirm_side;
                steer_start = now;
                want = steer_action(steer_side, 0);
                if (LOG_CHANGES)
                    printf("[%7lu ms] drift confirmed\n", (unsigned long)now);
            }
            // otherwise keep driving straight while waiting for the other sensor
            break;
        }

        case MODE_FOLLOW:
        default:
            if (left || right) // exactly one sensor (both was handled above)
            {
                int side = left ? -1 : +1;
                if (steer_side == side)
                {
                    // already steering this way: keep going, spin if it lasts too long
                    want = steer_action(side, now - steer_start);
                }
                else
                {
                    // A sensor has just reached black: drift, or the first edge
                    // of a junction? Drive straight a little and find out.
                    mode = MODE_CONFIRM;
                    confirm_side = side;
                    mark = enc_mark();
                    steer_side = 0;
                }
            }
            else
            {
                steer_side = 0; // line between the sensors
            }
            break;
        }

        check_barcode(); // decode barcodes in the background while driving

        if (want != current) // only touch the motors when the plan changes
        {
            apply(want);
            current = want;
            if (LOG_CHANGES)
                printf("[%7lu ms] %s\n", (unsigned long)now, action_name(want));
        }

        sleep_ms(LOOP_MS);
    }

    stopMotors();
    if (LOG_CHANGES)
        printf("junctions crossed: %lu\n", (unsigned long)junctions);
    while (button_down())
        sleep_ms(10); // wait for the button to be let go
    sleep_ms(30);
}

int main(void)
{
    stdio_init_all();
    ir_line_sensor_init(); // straight after stdio, so GP0 is an input again quickly
    wheel_init();          // motors stopped, encoders and speed sync running
    barcode_init();        // barcode sensor must be over white floor now (learns the floor level)

    gpio_init(START_BUTTON_PIN);
    gpio_set_dir(START_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(START_BUTTON_PIN);

    sleep_ms(2000); // time to open the serial monitor
    printf("\n=== line_follow built %s %s | straight=%d steer=%d/%d spin=%d after %d ms ===\n",
           __DATE__, __TIME__, STRAIGHT_SPEED, STEER_INNER, STEER_OUTER, SPIN_SPEED, SPIN_AFTER_MS);
    printf("    junction: confirm %d mm (%ld counts), cross %d mm (%ld counts), max %d mm\n",
           CONFIRM_MM, (long)MM_TO_COUNTS(CONFIRM_MM), CROSS_MM, (long)MM_TO_COUNTS(CROSS_MM), CROSS_MAX_MM);

    while (true)
    {
        printf("Put the car on the track with the line between the sensors, then press GP20.\n");
        wait_for_click();
        printf("Following - press GP20 to stop.\n");
        follow_line();
        printf("Stopped.\n\n");
    }
}
