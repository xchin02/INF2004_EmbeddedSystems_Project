// wheel.c - see wheel.h for settings and usage
#include "wheel.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "quadrature_encoder.pio.h" // generated from quadrature_encoder.pio at build time

#define PWM_FREQ_HZ 20000u // 20 kHz: above hearing range, fine for small DC motors
#define PWM_TOP 1000u      // duty levels 0..1000 (wrap = PWM_TOP - 1)
#define DRIVE_TIME_MS 1500 // how long each straight section lasts
#define PAUSE_MS 500       // pause between moves

// =========================================================
// Encoders - counted by two PIO state machines, no CPU time.
// The PIO keeps a running count; resetEncoders() just remembers
// the current value so the getters can return counts since reset.
// =========================================================
#define ENC_PIO pio0

static uint left_sm;
static uint right_sm;
static uint32_t left_zero = 0;
static uint32_t right_zero = 0;

static uint32_t left_raw(void)
{
    return (uint32_t)quadrature_encoder_get_count(ENC_PIO, left_sm);
}
static uint32_t right_raw(void)
{
    return (uint32_t)quadrature_encoder_get_count(ENC_PIO, right_sm);
}

// unsigned subtraction so it stays correct even if the raw count wraps around
int32_t getLeftCount(void)
{
    return (int32_t)(left_raw() - left_zero);
}
int32_t getRightCount(void)
{
    return (int32_t)(right_raw() - right_zero);
}

void resetEncoders(void)
{
    left_zero = left_raw();
    right_zero = right_raw();
}

static void encoders_init(void)
{
    // The program uses computed jumps, so it must sit at offset 0 of the PIO.
    // Both state machines share the one copy; each gets its own pins.
    pio_add_program(ENC_PIO, &quadrature_encoder_program);

    left_sm = (uint)pio_claim_unused_sm(ENC_PIO, true);
    right_sm = (uint)pio_claim_unused_sm(ENC_PIO, true);

    quadrature_encoder_program_init(ENC_PIO, left_sm, LEFT_ENC_A, LEFT_ENC_B);
    quadrature_encoder_program_init(ENC_PIO, right_sm, RIGHT_ENC_A, RIGHT_ENC_B);

    resetEncoders();
}

// =========================================================
// Motors - one PWM pin per direction (Robo Pico datasheet truth table)
//   forward: A = PWM, B = 0     reverse: A = 0, B = PWM
//   brake:   A = 0,   B = 0     coast:   A = 1, B = 1
// =========================================================
static void pwm_pin_init(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / (float)(PWM_FREQ_HZ * PWM_TOP));
    pwm_config_set_wrap(&cfg, PWM_TOP - 1);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(pin, 0);
}

static int clamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// speed_pm in 0.1% steps: -1000 (full reverse) .. 0 (brake) .. +1000 (full forward)
static void set_motor_pm(uint pin_a, uint pin_b, int speed_pm)
{
    speed_pm = clamp(speed_pm, -1000, 1000);
    uint16_t level = (uint16_t)((unsigned)abs(speed_pm) * PWM_TOP / 1000u);

    if (speed_pm > 0)
    {
        pwm_set_gpio_level(pin_a, level);
        pwm_set_gpio_level(pin_b, 0);
    }
    else if (speed_pm < 0)
    {
        pwm_set_gpio_level(pin_a, 0);
        pwm_set_gpio_level(pin_b, level);
    }
    else
    {
        pwm_set_gpio_level(pin_a, 0);
        pwm_set_gpio_level(pin_b, 0);
    }
}

// speed: -100 (full reverse) .. 0 (brake) .. +100 (full forward)
static void set_motor(uint pin_a, uint pin_b, int speed)
{
    set_motor_pm(pin_a, pin_b, clamp(speed, -100, 100) * 10);
}

// Both inputs LOW = brake on the Robo Pico: the motor stops quickly
static void brake_motor(uint pin_a, uint pin_b)
{
    pwm_set_gpio_level(pin_a, 0);
    pwm_set_gpio_level(pin_b, 0);
}

// Both inputs HIGH = coast on the Robo Pico: motor off, wheel turns freely
static void coast_motor(uint pin_a, uint pin_b)
{
    pwm_set_gpio_level(pin_a, PWM_TOP); // level > wrap = output stays high
    pwm_set_gpio_level(pin_b, PWM_TOP);
}

// =========================================================
// Straight / curved driving, kept in step by the encoders
// =========================================================
// While driving, a timer checks both encoders every SYNC_PERIOD_MS and
// nudges the wheel speeds so both wheels cover the same distance.
// With offsets (a curve), it keeps the ratio the offsets asked for instead.
#define SYNC_PERIOD_MS 10 // how often the speeds are corrected
#define SYNC_KP 0.5f      // correction per count of difference, in 0.1% speed steps
                          //   robot weaves side to side -> lower it
                          //   still drifts to one side  -> raise it
#define SYNC_MAX_CORR 150 // biggest correction allowed: 15% speed

static volatile bool drive_active = false; // true while moveForward()/reverse() is running
static volatile int drive_dir = 0;         // +1 forward, -1 reverse
static volatile int drive_left_pm = 0;     // commanded wheel speeds, 0..1000
static volatile int drive_right_pm = 0;
static volatile uint32_t drive_left_start = 0; // raw encoder counts when the drive started
static volatile uint32_t drive_right_start = 0;
static repeating_timer_t sync_timer;

// Runs in the background every SYNC_PERIOD_MS
static bool sync_callback(repeating_timer_t *t)
{
    (void)t;
    if (!drive_active)
    {
        return true; // not driving (stopped or turning) - leave the motors alone
    }

    // Distance each wheel has covered since the drive started
    float l = (float)abs((int32_t)(left_raw() - drive_left_start));
    float r = (float)abs((int32_t)(right_raw() - drive_right_start));
    float sl = (float)drive_left_pm;
    float sr = (float)drive_right_pm;

    // How many counts the left wheel is ahead of where it should be.
    // With equal speeds that's just left minus right (0 = driving straight).
    float ahead = (l * sr - r * sl) / ((sl + sr) * 0.5f);

    float corr = SYNC_KP * ahead;
    if (corr > SYNC_MAX_CORR)
        corr = SYNC_MAX_CORR;
    if (corr < -SYNC_MAX_CORR)
        corr = -SYNC_MAX_CORR;

    // Slow down the wheel that's ahead, speed up the one that's behind
    set_motor_pm(LEFT_MOTOR_A, LEFT_MOTOR_B, drive_dir * clamp((int)(sl - corr), 0, 1000));
    set_motor_pm(RIGHT_MOTOR_A, RIGHT_MOTOR_B, drive_dir * clamp((int)(sr + corr), 0, 1000));
    return true; // keep the timer running
}

static void start_drive(int dir, int leftOffset, int rightOffset)
{
    int left_pm = clamp(BASE_SPEED + leftOffset, 0, 100) * 10;
    int right_pm = clamp(BASE_SPEED + rightOffset, 0, 100) * 10;

    // Already driving like this (e.g. moveForward() called again inside a loop):
    // keep going without restarting the distance count
    if (drive_active && dir == drive_dir && left_pm == drive_left_pm && right_pm == drive_right_pm)
    {
        return;
    }

    drive_active = false; // pause the correction while setting up
    drive_dir = dir;
    drive_left_pm = left_pm;
    drive_right_pm = right_pm;
    drive_left_start = left_raw();
    drive_right_start = right_raw();

    set_motor_pm(LEFT_MOTOR_A, LEFT_MOTOR_B, dir * left_pm);
    set_motor_pm(RIGHT_MOTOR_A, RIGHT_MOTOR_B, dir * right_pm);

    drive_active = (left_pm + right_pm) > 0;
}

void moveForward(int leftOffset, int rightOffset)
{
    start_drive(+1, leftOffset, rightOffset);
}

void reverse(int leftOffset, int rightOffset)
{
    start_drive(-1, leftOffset, rightOffset);
}

void stopMotors(void)
{
    drive_active = false; // stop the straight-line correction first
    brake_motor(LEFT_MOTOR_A, LEFT_MOTOR_B);
    brake_motor(RIGHT_MOTOR_A, RIGHT_MOTOR_B);
}

// =========================================================
// Init
// =========================================================
void wheel_init(void)
{
    pwm_pin_init(LEFT_MOTOR_A);
    pwm_pin_init(LEFT_MOTOR_B);
    pwm_pin_init(RIGHT_MOTOR_A);
    pwm_pin_init(RIGHT_MOTOR_B);
    stopMotors();

    encoders_init();

    // Background speed correction for moveForward() / reverse()
    add_repeating_timer_ms(-SYNC_PERIOD_MS, sync_callback, NULL, &sync_timer);
}

// =========================================================
// Turns on the spot by a chosen angle (measured with the encoders)
// =========================================================
// Each wheel is driven to an exact encoder count:
//   - it speeds up gently (less wheel slip on the floor),
//   - slows down near the target and is held to a low speed as it arrives,
//   - brakes early by however far it will roll after braking
//     (measured after every turn, so it adapts to the floor and battery),
//   - gets extra power if friction stops it,
//   - and once the robot has stopped, any wheel that ended short or
//     went too far is nudged back to the target.
// So both wheels turn the same amount every time, whatever the friction.

// Encoder counts EACH wheel moves during a 90-degree turn on the spot.
// Calibrate once on the floor you'll use:
//   1. Run turn_left(90) and see what angle the robot actually turns.
//   2. New TURN_COUNTS_90 = TURN_COUNTS_90 x 90 / angle_you_got
//      e.g. 1000 turned 85 degrees -> 1000 x 90 / 85 = 1059
// Other angles scale from this automatically, e.g. turn_left(45) uses half.
#define TURN_COUNTS_90 1000

#define TURN_MIN_SPEED 20    // % - lowest power used near the target (raised if a wheel gets stuck)
#define TURN_RAMP_MS 150     // speed up from TURN_MIN_SPEED to TURN_SPEED over this time
#define TURN_SLOW_COUNTS 300 // start slowing down this many counts before the target
#define TURN_APPROACH_MS 100 // near the target, each wheel is kept slow enough to stop in about this time
#define TURN_CREEP_SPEED 250 // counts per second - the slowest that speed limit goes
#define TURN_TOLERANCE 5     // each wheel must finish within this many counts of the target
#define TURN_STALL_MS 50     // a wheel that hasn't moved for this long gets +5% power
#define TURN_SETTLE_MS 80    // wait this long after stopping before checking the result
#define TURN_FIX_TRIES 3     // how many times to nudge a wheel back to the target
#define TURN_TIMEOUT_MS 5000 // safety stop if a wheel is stuck or an encoder fails
#define SPEED_WINDOW_MS 5    // wheel speed is measured over this time

// How long each wheel keeps rolling after it brakes, in seconds.
// Starts as a guess and is re-measured after every turn.
static float brake_time_s[2] = {0.05f, 0.05f};

// Encoder counts each wheel must move for this many degrees
static int32_t turn_target_counts(int degrees)
{
    return (int32_t)((float)TURN_COUNTS_90 * (float)degrees / 90.0f + 0.5f);
}

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

// Everything the turn needs to know about one wheel
typedef struct
{
    uint pin_a, pin_b;
    int dir;                    // +1 = this wheel goes forward in this turn, -1 = backward
    int32_t (*get_count)(void); // getLeftCount or getRightCount
    float *brake_time;          // this wheel's measured braking time

    // working values, set up by drive_wheels_to()
    int32_t last_pos; // for spotting a stuck wheel
    uint32_t last_move_ms;
    int boost_pm; // extra power (0.1% steps) added when the wheel got stuck
    bool done;
    float speed; // measured speed, counts per second (+ = towards the target)
    int32_t speed_pos;
    uint32_t speed_ms;
    int32_t brake_pos; // where the wheel was when it braked (-1 = didn't brake)
    float brake_speed; // and how fast it was going
} turn_wheel_t;

// Power (0.1% steps) for a wheel that still has `remaining` counts to go
static int turn_speed_pm(int32_t remaining, uint32_t elapsed_ms, int boost_pm)
{
    int low = TURN_MIN_SPEED * 10 + boost_pm; // enough to keep the wheel moving
    int high = TURN_SPEED * 10;
    if (high < low)
        high = low;

    int speed = high;
    if (elapsed_ms < TURN_RAMP_MS) // gentle start
        speed = low + (high - low) * (int)elapsed_ms / TURN_RAMP_MS;

    if (remaining < TURN_SLOW_COUNTS) // slow down as the target gets close
    {
        int near = low + (high - low) * (int)remaining / TURN_SLOW_COUNTS;
        if (near < speed)
            speed = near;
    }
    return clamp(speed, 0, 1000);
}

// Gives a wheel more power if it hasn't moved for TURN_STALL_MS (friction)
static void check_stuck(turn_wheel_t *w, int32_t pos, uint32_t now)
{
    if (pos != w->last_pos)
    {
        w->last_pos = pos;
        w->last_move_ms = now;
    }
    else if (now - w->last_move_ms > TURN_STALL_MS)
    {
        w->boost_pm = clamp(w->boost_pm + 50, 0, 500); // +5%, at most +50%
        w->last_move_ms = now;                         // give it time to react
    }
}

// Drives both wheels until each is within TURN_TOLERANCE of the target.
// Works in both directions, so it also backs up a wheel that went too far.
// Returns false if the deadline passed first.
static bool drive_wheels_to(turn_wheel_t w[2], int32_t target, bool ramp_up, uint32_t deadline)
{
    uint32_t start = now_ms();
    for (int i = 0; i < 2; i++)
    {
        int32_t pos = abs(w[i].get_count());
        w[i].done = false;
        w[i].last_pos = pos;
        w[i].last_move_ms = start;
        w[i].speed = 0.0f;
        w[i].speed_pos = pos;
        w[i].speed_ms = start;
        w[i].brake_pos = -1;
        w[i].brake_speed = 0.0f;
    }

    while (!(w[0].done && w[1].done))
    {
        uint32_t now = now_ms();
        if ((int32_t)(now - deadline) >= 0)
        {
            return false;
        }

        for (int i = 0; i < 2; i++)
        {
            turn_wheel_t *wh = &w[i];
            if (wh->done)
                continue;

            int32_t pos = abs(wh->get_count()); // abs() so encoder direction doesn't matter
            int32_t error = target - pos;       // + = still short, - = gone too far

            // Measure the wheel's speed every SPEED_WINDOW_MS
            if (now - wh->speed_ms >= SPEED_WINDOW_MS)
            {
                wh->speed = (float)(pos - wh->speed_pos) * 1000.0f / (float)(now - wh->speed_ms);
                wh->speed_pos = pos;
                wh->speed_ms = now;
            }

            // Where the wheel would end up if it braked right now
            float stop_error = (float)target - ((float)pos + wh->speed * *wh->brake_time);

            if (abs(error) <= TURN_TOLERANCE ||
                (error > 0 && stop_error <= TURN_TOLERANCE) ||
                (error < 0 && stop_error >= -TURN_TOLERANCE))
            {
                brake_motor(wh->pin_a, wh->pin_b); // it will roll the rest of the way
                wh->done = true;
                wh->brake_pos = pos;
                wh->brake_speed = wh->speed;
                continue;
            }

            // Near the target, keep the wheel slow so it always arrives gently
            float speed_limit = (float)abs(error) * 1000.0f / TURN_APPROACH_MS;
            if (speed_limit < TURN_CREEP_SPEED)
                speed_limit = TURN_CREEP_SPEED;
            if (abs(error) < TURN_SLOW_COUNTS && fabsf(wh->speed) > speed_limit)
            {
                brake_motor(wh->pin_a, wh->pin_b); // too fast - brake for a moment
                wh->last_move_ms = now;            // it isn't stuck, just slowing down
                continue;
            }

            check_stuck(wh, pos, now);
            int power = turn_speed_pm(abs(error), ramp_up ? now - start : TURN_RAMP_MS, wh->boost_pm);
            int dir = error > 0 ? wh->dir : -wh->dir; // went too far -> back up
            set_motor_pm(wh->pin_a, wh->pin_b, dir * power);
        }
    }
    return true;
}

// Once the robot has stopped: see how far each wheel rolled after braking,
// and update its braking time for next time
static void learn_brake_time(turn_wheel_t w[2])
{
    for (int i = 0; i < 2; i++)
    {
        if (w[i].brake_pos < 0 || fabsf(w[i].brake_speed) < 300.0f)
            continue; // didn't brake, or was too slow to measure

        float rolled = fabsf((float)(abs(w[i].get_count()) - w[i].brake_pos));
        float sample = rolled / fabsf(w[i].brake_speed);
        float t = 0.7f * *w[i].brake_time + 0.3f * sample;
        if (t < 0.01f)
            t = 0.01f;
        if (t > 0.2f)
            t = 0.2f;
        *w[i].brake_time = t;
    }
}

static bool both_on_target(turn_wheel_t w[2], int32_t target)
{
    return abs(target - abs(w[0].get_count())) <= TURN_TOLERANCE &&
           abs(target - abs(w[1].get_count())) <= TURN_TOLERANCE;
}

// left_dir / right_dir: +1 = wheel forward, -1 = wheel backward.
// Runs once and returns only after the robot has finished turning and stopped.
static void spin_on_spot(int left_dir, int right_dir, int degrees)
{
    if (degrees <= 0)
    {
        return; // nothing to do
    }

    const int32_t target = turn_target_counts(degrees);
    turn_wheel_t wheels[2] = {
        {.pin_a = LEFT_MOTOR_A, .pin_b = LEFT_MOTOR_B, .dir = left_dir, .get_count = getLeftCount, .brake_time = &brake_time_s[0]},
        {.pin_a = RIGHT_MOTOR_A, .pin_b = RIGHT_MOTOR_B, .dir = right_dir, .get_count = getRightCount, .brake_time = &brake_time_s[1]},
    };

    stopMotors();
    sleep_ms(50); // start from standstill so every turn is the same
    resetEncoders();
    uint32_t deadline = now_ms() + TURN_TIMEOUT_MS;

    // Main move
    bool ok = drive_wheels_to(wheels, target, true, deadline);

    // Once the robot has stopped, nudge any wheel that ended short or went too far
    for (int attempt = 0; ok && attempt < TURN_FIX_TRIES; attempt++)
    {
        sleep_ms(TURN_SETTLE_MS);
        learn_brake_time(wheels);
        if (both_on_target(wheels, target))
            break;
        ok = drive_wheels_to(wheels, target, false, deadline);
    }

    stopMotors();
    if (!ok)
    {
        printf("turn timed out - a wheel is stuck or an encoder isn't counting\n");
    }
    sleep_ms(100); // let the robot settle before the next move
}

void turn_right(int degrees)
{
    spin_on_spot(+1, -1, degrees); // left wheel forward, right wheel backward
}

void turn_left(int degrees)
{
    spin_on_spot(-1, +1, degrees); // left wheel backward, right wheel forward
}

// =========================================================
// Calibration: motors off, prints both encoders forever.
// Call after stdio_init_all() and wheel_init(). Never returns.
//  - A/B should flip between 0 and 1 as you turn a wheel by hand.
//  - Turn a wheel exactly ONE full revolution: the count you get
//    (ignore any minus sign) is your encoder counts per wheel turn.
// =========================================================
void encoderCheck(void)
{
    // Motors off (coasting) so the wheels turn freely by hand
    drive_active = false;
    coast_motor(LEFT_MOTOR_A, LEFT_MOTOR_B);
    coast_motor(RIGHT_MOTOR_A, RIGHT_MOTOR_B);

    sleep_ms(2000); // time to open the serial monitor
    printf("Encoder check: turn each wheel ONE full revolution by hand.\n");
    resetEncoders();

    while (true)
    {
        printf("LEFT  A=%d B=%d count=%6ld   |   RIGHT  A=%d B=%d count=%6ld\n",
               gpio_get(LEFT_ENC_A), gpio_get(LEFT_ENC_B), (long)getLeftCount(),
               gpio_get(RIGHT_ENC_A), gpio_get(RIGHT_ENC_B), (long)getRightCount());
        sleep_ms(200);
    }
}

// Stop, print the encoder counts over USB, then pause
static void stop_and_report(const char *step)
{
    stopMotors();
    printf("%-8s left=%ld right=%ld\n", step, (long)getLeftCount(), (long)getRightCount());
    sleep_ms(PAUSE_MS);
}

int main(void)
{
    stdio_init_all();
    wheel_init();
    // encoderCheck();   // uncomment for encoder calibration mode (never returns)

    sleep_ms(3000); // time to put the robot down (and open the serial monitor)

    // Shows which build is running and with which settings.
    // If the date/time here isn't your latest build, the Pico has old code on it.
    printf("\n=== motion.c built %s %s | BASE_SPEED=%d TURN_SPEED=%d TURN_COUNTS_90=%d ===\n",
           __DATE__, __TIME__, BASE_SPEED, TURN_SPEED, TURN_COUNTS_90);

    // Every step below finishes completely before the next one starts.

    // 1. Forward - BOTH wheels should roll forward. If the robot spins
    //    here instead, one motor is reversed: swap its _A/_B pins in wheel.h.
    printf("1. forward %d ms\n", DRIVE_TIME_MS);
    resetEncoders();
    moveForward(0, 0);
    sleep_ms(DRIVE_TIME_MS);
    stop_and_report("forward");

    // 2. Turn left 90 degrees - main() waits here until the turn is done
    printf("2. turn_left(90): target %ld counts per wheel\n", (long)turn_target_counts(90));
    turn_left(90);
    stop_and_report("left");

    // 3. Forward again
    printf("3. forward %d ms\n", DRIVE_TIME_MS);
    resetEncoders();
    moveForward(0, 0);
    sleep_ms(DRIVE_TIME_MS);
    stop_and_report("forward");

    // 4. Turn right 90 degrees - main() waits here until the turn is done
    printf("4. turn_right(90): target %ld counts per wheel\n", (long)turn_target_counts(90));
    turn_right(90);
    stop_and_report("right");

    // 5. Reverse
    printf("5. reverse %d ms\n", DRIVE_TIME_MS);
    resetEncoders();
    reverse(0, 0);
    sleep_ms(DRIVE_TIME_MS);
    stop_and_report("reverse");

    printf("done\n");
    while (true)
    {
        sleep_ms(1000); // route finished - stay stopped
    }
}