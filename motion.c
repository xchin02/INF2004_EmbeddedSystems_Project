/**
 * Robo Pico + Pico W - two-wheel robot movement, all in one file.
 *
 *   Left wheel : Motor 2 (GP10/GP11), encoder on Grove 4 (GP16/GP17)
 *   Right wheel: Motor 1 (GP8/GP9),   encoder on Grove 5 (GP6/GP26)
 *
 * Movement functions:
 *   move_forward(speed, offset_left, offset_right)
 *   move_reverse(speed, offset_left, offset_right)
 *   turn_left (speed, offset_left, offset_right)    keeps turning on the spot
 *   turn_right(speed, offset_left, offset_right)    keeps turning on the spot
 *   spin_left (speed, offset_left, offset_right)    turns exactly 90 deg, stops
 *   spin_right(speed, offset_left, offset_right)    turns exactly 90 deg, stops
 *   stop_motors()
 *   get_speed() cm/s, get_distance() cm, reset_distance(), get_wheel_speeds() cm/s
 *
 * speed  : 0..100 (% motor power)
 *
 * AUTO BALANCE: while any move is running, a timer checks the encoders
 * every 10 ms and moves power from the wheel that is ahead to the one that
 * is behind, so both wheels always turn at the same speed. The robot drives
 * straight and spins on the spot without any tuning.
 *
 * offsets: extra power for that wheel (% too), whichever way it spins.
 *          With AUTO_BALANCE on they are optional: they only give the
 *          balancing a head start (it settles a bit quicker). Leave them 0.
 *          With AUTO_BALANCE 0 they are the only thing matching the wheels:
 *          give the faster wheel a small negative offset.
 */
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

/* ======================= pins =======================
 * Left / right = as seen from behind the robot, facing its front.
 * On this robot the Motor 2 wheel is on the LEFT and the Motor 1 wheel is
 * on the RIGHT. Check with ENCODER_CHECK: rolling the LEFT wheel by hand
 * must change only the "left" count.                                    */
#define LEFT_MOTOR_A 11 /* Motor 2: M2B - A and B swapped on    */
#define LEFT_MOTOR_B 10 /* Motor 2: M2A   purpose, so both       */
#define RIGHT_MOTOR_A 9 /* Motor 1: M1B   motors drive the robot */
#define RIGHT_MOTOR_B 8 /* Motor 1: M1A   forwards               */

#define LEFT_ENC_A 16 /* Grove 4 = left wheel's encoder        */
#define LEFT_ENC_B 17
#define RIGHT_ENC_A 6 /* Grove 5 = right wheel's encoder (GP26 \
                         is shared with Grove 6: keep it free) */
#define RIGHT_ENC_B 26

/* MOTOR_INVERT: set to 1 if that wheel spins backwards on move_forward()
 *               (same effect as swapping its A and B pins above).
 * ENC_INVERT:   set to 1 if that wheel's speed shows NEGATIVE while it
 *               rolls forwards. Changing a motor's direction does NOT change
 *               its encoder - check these separately in the serial monitor. */
#define LEFT_MOTOR_INVERT 0
#define RIGHT_MOTOR_INVERT 1
#define LEFT_ENC_INVERT 1
#define RIGHT_ENC_INVERT 0

/* ============ robot measurements (all in cm) ============ */
#define COUNTS_PER_REV (11 * 4 * 30) /* encoder PPR x 4 x gear ratio  */
#define WHEEL_DIAMETER_CM 6.5f
#define TRACK_WIDTH_CM 15.0f /* centre of left tyre to centre \
                                of right tyre                */
#define CM_PER_COUNT (3.14159265f * WHEEL_DIAMETER_CM / COUNTS_PER_REV)

/* ===================== turning ===================== */
#define SPIN_ANGLE_DEG 90.0f     /* spin_left / spin_right turn this much */
#define TURN_CALIBRATION 1.20f   /* Tyres slip a little in a spin, so the      \
                                    wheels must roll further than the maths    \
                                    says. Set from your test (~75 deg for 90). \
                                    Turns too little: make it bigger.          \
                                    Turns too much: make it smaller.           \
                                    new value = old value x 90 / real angle */
#define TURN_SLOW_DEG 20.0f      /* slow down for the last 20 degrees     */
#define TURN_SLOW_SPEED 25       /* % power for that slow part (must be \
                                    enough to keep the wheels turning)    */
#define TURN_STOP_EARLY_DEG 1.5f /* cut power this much early, the robot \
                                    rolls the rest. Turns too far: raise \
                                    it. Stops short: lower it.            */
#define TURN_TIMEOUT_MS 5000     /* give up if the turn takes longer      */

/* ================== auto balance ================== */
#define AUTO_BALANCE 1       /* 1 = encoders keep both wheels at the \
                                same speed, 0 = off (plain power)     */
#define BALANCE_PERIOD_MS 10 /* how often it corrects                 */
#define BALANCE_KP 0.15f     /* % power per count one wheel is ahead. \
                                Wobbles side to side: lower it.       \
                                Corrects too slowly: raise it.        */
#define BALANCE_KI 0.30f     /* removes the last bit of difference    */
#define BALANCE_MAX 30.0f    /* most power it may move between wheels */

#define PWM_FREQ_HZ 20000  /* Robo Pico motor driver max            */
#define SPEED_SAMPLE_MS 50 /* how often speed is measured           */

/* =====================================================================
 *  Motors
 *  Robo Pico driver: A=PWM,B=LOW forward | A=LOW,B=PWM backward |
 *                    A=LOW,B=LOW brake
 * ===================================================================== */
static uint16_t pwm_top;

static void motors_init(void)
{
    pwm_top = (uint16_t)(clock_get_hz(clk_sys) / PWM_FREQ_HZ - 1); /* 6249 at 125 MHz */

    const uint pins[4] = {LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B};
    for (int i = 0; i < 4; i++)
    {
        gpio_set_function(pins[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pins[i]);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_config_set_wrap(&cfg, pwm_top);
        pwm_init(slice, &cfg, true);
        pwm_set_gpio_level(pins[i], 0); /* all LOW = brake */
    }
}

/* power: -100 (full backwards) .. 0 (brake) .. +100 (full forwards).
 * float, so the balancing can make changes finer than 1%. */
static void set_motor(uint pin_a, uint pin_b, float power, bool invert)
{
    if (invert)
        power = -power;
    if (power > 100.0f)
        power = 100.0f;
    if (power < -100.0f)
        power = -100.0f;

    float mag = power < 0.0f ? -power : power;
    uint16_t level = (uint16_t)(mag * (float)(pwm_top + 1) / 100.0f);

    if (power > 0.0f)
    {
        pwm_set_gpio_level(pin_a, level);
        pwm_set_gpio_level(pin_b, 0);
    }
    else if (power < 0.0f)
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

static void set_wheels(float left, float right)
{
    set_motor(LEFT_MOTOR_A, LEFT_MOTOR_B, left, LEFT_MOTOR_INVERT);
    set_motor(RIGHT_MOTOR_A, RIGHT_MOTOR_B, right, RIGHT_MOTOR_INVERT);
}

/* keep a power between 0 and 100 */
static float limit_power(float p)
{
    if (p < 0.0f)
        p = 0.0f;
    if (p > 100.0f)
        p = 100.0f;
    return p;
}

/* =====================================================================
 *  Encoders (quadrature, 4x decoding in a GPIO interrupt)
 * ===================================================================== */
static volatile int32_t left_count, right_count;
static volatile uint8_t left_state, right_state;

/* (old state << 2 | new state) -> +1 forward, -1 backward, 0 none */
static const int8_t QDEC[16] = {0, -1, +1, 0, +1, 0, 0, -1, -1, 0, 0, +1, 0, +1, -1, 0};

/* Runs on every edge of any encoder pin.
 * (Only ONE GPIO callback is allowed: if you add other GPIO interrupts
 *  later, e.g. an ultrasonic echo pin, handle them in here too.) */
static void gpio_callback(uint gpio, uint32_t events)
{
    (void)gpio;
    (void)events;
    uint32_t pins = gpio_get_all();
    uint8_t l = (uint8_t)((((pins >> LEFT_ENC_A) & 1u) << 1) | ((pins >> LEFT_ENC_B) & 1u));
    uint8_t r = (uint8_t)((((pins >> RIGHT_ENC_A) & 1u) << 1) | ((pins >> RIGHT_ENC_B) & 1u));
    left_count += QDEC[(left_state << 2) | l];
    left_state = l;
    right_count += QDEC[(right_state << 2) | r];
    right_state = r;
}

static int32_t left_counts(void) { return LEFT_ENC_INVERT ? -left_count : left_count; }
static int32_t right_counts(void) { return RIGHT_ENC_INVERT ? -right_count : right_count; }

/* ---------- speed: measured every SPEED_SAMPLE_MS by a timer ---------- */
static volatile float left_speed, right_speed; /* cm/s */
static struct repeating_timer speed_timer;

static bool speed_timer_cb(struct repeating_timer *t)
{
    (void)t;
    static int32_t last_l, last_r;
    int32_t l = left_counts(), r = right_counts();
    left_speed = (float)(l - last_l) * CM_PER_COUNT * (1000.0f / SPEED_SAMPLE_MS);
    right_speed = (float)(r - last_r) * CM_PER_COUNT * (1000.0f / SPEED_SAMPLE_MS);
    last_l = l;
    last_r = r;
    return true;
}

static void encoders_init(void)
{
    const uint pins[4] = {LEFT_ENC_A, LEFT_ENC_B, RIGHT_ENC_A, RIGHT_ENC_B};
    for (int i = 0; i < 4; i++)
    {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }
    sleep_us(100);
    uint32_t p = gpio_get_all();
    left_state = (uint8_t)((((p >> LEFT_ENC_A) & 1u) << 1) | ((p >> LEFT_ENC_B) & 1u));
    right_state = (uint8_t)((((p >> RIGHT_ENC_A) & 1u) << 1) | ((p >> RIGHT_ENC_B) & 1u));

    const uint32_t edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    gpio_set_irq_enabled_with_callback(pins[0], edges, true, &gpio_callback);
    for (int i = 1; i < 4; i++)
        gpio_set_irq_enabled(pins[i], edges, true);

    add_repeating_timer_ms(-SPEED_SAMPLE_MS, speed_timer_cb, NULL, &speed_timer);
}

/* =====================================================================
 *  Speed and distance
 * ===================================================================== */
static int32_t dist_start_l, dist_start_r;

/* robot speed in cm/s: + forwards, - backwards */
float get_speed(void)
{
    return 0.5f * (left_speed + right_speed);
}

/* each wheel's speed in cm/s - handy for tuning offsets */
void get_wheel_speeds(float *left_cm_s, float *right_cm_s)
{
    *left_cm_s = left_speed;
    *right_cm_s = right_speed;
}

/* distance in cm since reset_distance(): + forwards, - backwards */
float get_distance(void)
{
    int32_t dl = left_counts() - dist_start_l;
    int32_t dr = right_counts() - dist_start_r;
    return 0.5f * (float)(dl + dr) * CM_PER_COUNT;
}

void reset_distance(void)
{
    dist_start_l = left_counts();
    dist_start_r = right_counts();
}

/* =====================================================================
 *  Movement + auto balance
 *
 *  A move = a direction for each wheel + a power. While it runs, a timer
 *  every BALANCE_PERIOD_MS compares how far each wheel has turned since the
 *  move started. If one is ahead, it gets a little less power and the other
 *  a little more, until both have turned the same amount. Equal distance
 *  at every moment = equal speed, so the robot keeps a straight line (and
 *  undoes any drift from the start), and spins stay on the spot.
 * ===================================================================== */
static volatile bool mv_active;           /* a move is running             */
static volatile int mv_dir_l, mv_dir_r;   /* +1 forwards, -1 backwards     */
static volatile float mv_power;           /* base power %                  */
static volatile float mv_off_l, mv_off_r; /* offsets %                     */
static int32_t mv_start_l, mv_start_r;

static float bal_integral;
static int32_t bal_err_forgotten;      /* distance gap it gave up on     */
static volatile float bal_correction;  /* % power moved to the right
                                          wheel (+) or to the left (-)  */
static volatile bool bal_off_for_move; /* a wheel isn't turning at all  */

/* What's going on when balancing is at its limit (for the printout) */
enum
{
    BAL_OK = 0,
    BAL_LEFT_HELD,
    BAL_RIGHT_HELD,
    BAL_CANT_MATCH
};
static volatile int bal_status;

/* Short history of each wheel's progress, to measure their speeds
 * over the last 150 ms (for bal_status). */
#define BAL_HIST 15 /* 15 x 10 ms = 150 ms           */
static int32_t bal_hist_l[BAL_HIST], bal_hist_r[BAL_HIST];
static int bal_hist_i;
static int bal_max_ticks; /* time at the limit (ticks)     */
static struct repeating_timer balance_timer;

/* Set the motor outputs for the current move (+ balance correction) */
static void apply_move(void)
{
    float c = bal_correction;
    float l = limit_power(mv_power + mv_off_l - c / 2.0f);
    float r = limit_power(mv_power + mv_off_r + c / 2.0f);
    set_wheels(mv_dir_l * l, mv_dir_r * r);
}

/* The balancing step - runs every BALANCE_PERIOD_MS in a timer interrupt */
static bool balance_timer_cb(struct repeating_timer *t)
{
    (void)t;
    if (!mv_active)
        return true;

    /* How far each wheel has turned since the move started, in encoder
     * counts. abs(): only the amount matters, so an encoder that counts
     * the wrong way can't upset the balancing. */
    int32_t pl = abs(left_counts() - mv_start_l);
    int32_t pr = abs(right_counts() - mv_start_r);

    /* One encoder not counting at all (unplugged)? Then balancing would
     * pour all the power into that wheel - switch it off for this move. */
    int32_t big = pl > pr ? pl : pr;
    int32_t small = pl > pr ? pr : pl;
    if (big > 150 && small * 4 < big)
        bal_off_for_move = true;

    /* speed of each wheel over the last 150 ms (in counts per 150 ms) */
    int32_t vl = pl - bal_hist_l[bal_hist_i];
    int32_t vr = pr - bal_hist_r[bal_hist_i];
    bal_hist_l[bal_hist_i] = pl;
    bal_hist_r[bal_hist_i] = pr;
    bal_hist_i = (bal_hist_i + 1) % BAL_HIST;

    if (AUTO_BALANCE && !bal_off_for_move)
    {
        const float dt = BALANCE_PERIOD_MS * 0.001f;
        int32_t raw_err = pl - pr; /* + = left wheel is ahead */

        /* If a wheel was held back, don't try to make up more than
         * ERR_CAP counts (~4.5 cm) once it's free - that would be a lurch. */
        const int32_t ERR_CAP = (int32_t)(1.5f * BALANCE_MAX / BALANCE_KP);
        int32_t err_i = raw_err - bal_err_forgotten;
        if (err_i > ERR_CAP)
        {
            bal_err_forgotten += err_i - ERR_CAP;
            err_i = ERR_CAP;
        }
        if (err_i < -ERR_CAP)
        {
            bal_err_forgotten += err_i + ERR_CAP;
            err_i = -ERR_CAP;
        }
        float err = (float)err_i;

        /* PI correction. The integral only grows while there's room to
         * correct (anti-windup), so it can't build up behind a stuck wheel. */
        float i_new = bal_integral + err * dt;
        float c = BALANCE_KP * err + BALANCE_KI * i_new;
        if ((c < BALANCE_MAX && c > -BALANCE_MAX) ||
            (c >= BALANCE_MAX && err < 0.0f) || (c <= -BALANCE_MAX && err > 0.0f))
            bal_integral = i_new;
        c = BALANCE_KP * err + BALANCE_KI * bal_integral;
        if (c > BALANCE_MAX)
            c = BALANCE_MAX;
        if (c < -BALANCE_MAX)
            c = -BALANCE_MAX;
        bal_correction = c;

        /* At its limit for a while: work out why, for the printout.
         * Balancing keeps doing its best either way. */
        if (c >= BALANCE_MAX || c <= -BALANCE_MAX)
        {
            if (++bal_max_ticks >= BAL_HIST)
            {                                            /* 150 ms at the limit */
                int32_t lead_v = raw_err > 0 ? vl : vr;  /* wheel that's ahead  */
                int32_t trail_v = raw_err > 0 ? vr : vl; /* wheel that's behind */
                if (trail_v * 5 < lead_v)
                    /* the trailing wheel gets MORE power but barely turns:
                     * something is holding it (hand, obstacle, cable) */
                    bal_status = raw_err > 0 ? BAL_RIGHT_HELD : BAL_LEFT_HELD;
                else
                    /* both turning but still can't be matched: a badly
                     * dragging wheel, or encoders swapped left/right */
                    bal_status = BAL_CANT_MATCH;
            }
        }
        else
        {
            bal_max_ticks = 0;
            bal_status = BAL_OK;
        }
    }
    else
    {
        bal_correction = 0.0f;
        bal_status = BAL_OK;
    }

    apply_move();
    return true;
}

/* Start (or update) a move. If the same kind of move is already running
 * (e.g. move_forward called again with a new power) the balancing carries
 * on; a different move, or restart = true, starts it fresh. */
static void start_move(int dir_l, int dir_r, float power, float off_l, float off_r, bool restart)
{
    uint32_t irq = save_and_disable_interrupts(); /* timer must not run halfway */

    bool same = mv_active && dir_l == mv_dir_l && dir_r == mv_dir_r;
    if (restart || !same)
    {
        mv_start_l = left_counts();
        mv_start_r = right_counts();
        bal_integral = 0.0f;
        bal_err_forgotten = 0;
        bal_correction = 0.0f;
        bal_off_for_move = false;
        bal_status = BAL_OK;
        bal_max_ticks = 0;
        for (int i = 0; i < BAL_HIST; i++)
            bal_hist_l[i] = bal_hist_r[i] = 0;
    }
    mv_dir_l = dir_l;
    mv_dir_r = dir_r;
    mv_power = power;
    mv_off_l = off_l;
    mv_off_r = off_r;
    mv_active = true;
    apply_move();

    restore_interrupts(irq);
}

static void balance_init(void)
{
    add_repeating_timer_ms(-BALANCE_PERIOD_MS, balance_timer_cb, NULL, &balance_timer);
}

void stop_motors(void)
{
    uint32_t irq = save_and_disable_interrupts();
    mv_active = false;
    set_wheels(0.0f, 0.0f); /* brake */
    restore_interrupts(irq);
}

/* Both wheels forwards. Keeps going until stop_motors() or another move. */
void move_forward(int speed, int offset_left, int offset_right)
{
    start_move(+1, +1, (float)speed, (float)offset_left, (float)offset_right, false);
}

/* Both wheels backwards. Keeps going until stop_motors() or another move. */
void move_reverse(int speed, int offset_left, int offset_right)
{
    start_move(-1, -1, (float)speed, (float)offset_left, (float)offset_right, false);
}

/* Turn left on the spot (left wheel back, right wheel forward).
 * Keeps turning until stop_motors() or another move. */
void turn_left(int speed, int offset_left, int offset_right)
{
    start_move(-1, +1, (float)speed, (float)offset_left, (float)offset_right, false);
}

/* Turn right on the spot (left wheel forward, right wheel back).
 * Keeps turning until stop_motors() or another move. */
void turn_right(int speed, int offset_left, int offset_right)
{
    start_move(+1, -1, (float)speed, (float)offset_left, (float)offset_right, false);
}

/* What the last 90-degree spin measured (printed by main) */
static int32_t last_spin_left_counts, last_spin_right_counts;
static float last_spin_deg;

/* dir = +1 left, -1 right. Spins on the spot until the encoders say the
 * robot has turned angle_deg, then brakes. Returns false on timeout. */
static bool turn(int dir, float angle_deg, int speed, int offset_left, int offset_right)
{
    const float RAD_TO_DEG = 57.29578f;
    int32_t l0 = left_counts(), r0 = right_counts();
    absolute_time_t give_up = make_timeout_time_ms(TURN_TIMEOUT_MS);
    bool finished = false;

    /* left turn: left wheel back, right wheel forward (balanced like any move) */
    start_move(-dir, dir, (float)speed, (float)offset_left, (float)offset_right, true);

    while (true)
    {
        /* In a spin both wheels roll the same distance in opposite
         * directions, so: angle = (left distance + right distance) / track width.
         * Using how FAR each wheel moved (not which way) means an encoder
         * counting backwards can't stop the spin from ending. */
        int32_t dl = abs(left_counts() - l0);
        int32_t dr = abs(right_counts() - r0);

        int32_t big = dl > dr ? dl : dr;
        int32_t small = dl > dr ? dr : dl;
        float wheel_counts = (float)(dl + dr);
        if (big > 100 && small * 4 < big)     /* one encoder isn't counting: */
            wheel_counts = 2.0f * (float)big; /* trust the one that is      */

        float turned = wheel_counts * CM_PER_COUNT / (TRACK_WIDTH_CM * TURN_CALIBRATION) * RAD_TO_DEG;
        float remaining = angle_deg - turned;

        last_spin_left_counts = left_counts() - l0;
        last_spin_right_counts = right_counts() - r0;
        last_spin_deg = turned;

        if (remaining <= TURN_STOP_EARLY_DEG)
        {
            finished = true;
            break;
        }
        if (time_reached(give_up))
            break;

        if (remaining < TURN_SLOW_DEG && speed > TURN_SLOW_SPEED)
            mv_power = (float)TURN_SLOW_SPEED; /* slow down near the end = more exact */
        sleep_ms(1);
    }

    stop_motors();
    sleep_ms(150); /* let it come to rest */
    return finished;
}

/* Spin LEFT exactly 90 degrees on the spot (left wheel back, right wheel
 * forward), then stop. Waits until the turn is finished.
 * Returns false if it didn't finish within TURN_TIMEOUT_MS. */
bool spin_left(int speed, int offset_left, int offset_right)
{
    return turn(+1, SPIN_ANGLE_DEG, speed, offset_left, offset_right);
}

/* Spin RIGHT exactly 90 degrees on the spot (left wheel forward, right
 * wheel back), then stop. Waits until the turn is finished. */
bool spin_right(int speed, int offset_left, int offset_right)
{
    return turn(-1, SPIN_ANGLE_DEG, speed, offset_left, offset_right);
}

/* =====================================================================
 *  main
 * ===================================================================== */
static void print_status(const char *label)
{
    float l, r;
    get_wheel_speeds(&l, &r);
    printf("%-12s speed=%5.1f cm/s  distance=%6.1f cm   (left %5.1f / right %5.1f cm/s)",
           label, get_speed(), get_distance(), l, r);
    if (!mv_active || !AUTO_BALANCE)
        printf("\n");
    else if (bal_off_for_move)
        printf("  BALANCE OFF this move: a wheel isn't turning (stuck, or encoder unplugged)\n");
    else if (bal_status == BAL_LEFT_HELD || bal_status == BAL_RIGHT_HELD)
        printf("  balance %+5.1f%%  %s wheel held back?\n", bal_correction,
               bal_status == BAL_LEFT_HELD ? "LEFT" : "RIGHT");
    else if (bal_status == BAL_CANT_MATCH)
        printf("  balance %+5.1f%%  can't match the wheels: one dragging, or encoders swapped?\n",
               bal_correction);
    else
        printf("  balance %+5.1f%%\n", bal_correction); /* + = extra power to the right wheel */
}

/* Keep doing the current move for ms milliseconds, printing twice a second */
static void keep_going(const char *label, uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += 500)
    {
        print_status(label);
        sleep_ms(500);
    }
}

/* What the encoders saw during the last 90-degree spin */
static void print_spin(const char *label, bool finished)
{
    printf("%s: %s  measured %.1f deg  (left wheel %+ld counts, right wheel %+ld counts)\n",
           label, finished ? "done" : "TIMED OUT",
           last_spin_deg, (long)last_spin_left_counts, (long)last_spin_right_counts);
    if (!finished)
        printf("   -> the encoders barely counted: check their wiring and pins\n");
}

/* Set to 1 to check the encoders by hand: motors stay off, counts are printed.
 * Roll each wheel FORWARD exactly one full turn: its count should go UP by
 * about COUNTS_PER_REV. */
#define ENCODER_CHECK 0

/* Set to 1 to tune TURN_CALIBRATION: the robot does 4 left spins (should be
 * one full circle, ending facing exactly where it started), then 4 right.
 * Put a strip of tape on the floor under its nose to see the difference. */
#define SPIN_TEST 0

int main(void)
{
    stdio_init_all();
    motors_init();
    encoders_init();
    balance_init(); /* starts the 10 ms auto-balance timer */

    sleep_ms(3000); /* time to open the serial monitor */

#if ENCODER_CHECK
    printf("\nENCODER CHECK: roll each wheel forward one full turn by hand.\n");
    printf("Roll the LEFT wheel: only the left count may change, and it should go\n");
    printf("UP by about %d. Then the same for the RIGHT wheel.\n", COUNTS_PER_REV);
    while (true)
    {
        printf("left %+7ld   right %+7ld\n", (long)left_counts(), (long)right_counts());
        sleep_ms(300);
    }
#endif

#if SPIN_TEST
    printf("\nSPIN TEST: 4 x left 90 (one full circle), then 4 x right 90\n");
    for (int i = 1; i <= 4; i++)
    {
        print_spin("spin left 90 ", spin_left(40, 0, 0));
        sleep_ms(700);
    }
    printf("Now facing the start? Short: raise TURN_CALIBRATION. Past it: lower it.\n");
    sleep_ms(3000);
    for (int i = 1; i <= 4; i++)
    {
        print_spin("spin right 90", spin_right(40, 0, 0));
        sleep_ms(700);
    }
    printf("Done.\n");
    while (true)
        sleep_ms(1000);
#endif

    printf("\nRobot movement sequence\n");
    reset_distance();

    /* Forward 10 s */
    move_forward(40, 0, 0); /* 40% power, no offsets */
    keep_going("forward", 10000);
    stop_motors();
    sleep_ms(500); /* short pause between moves */

    /* Spin left 90 degrees */
    print_spin("spin left 90", spin_left(40, 0, 0));
    sleep_ms(500);

    /* Spin right 90 degrees */
    print_spin("spin right 90", spin_right(40, 0, 0));
    sleep_ms(500);

    /* Turn left 5 s with offsets (keeps turning until stopped) */
    turn_left(40, 0, 0); /* power, left offset, right offset */
    keep_going("turn left", 5000);
    stop_motors();
    sleep_ms(500);

    /* Turn right 5 s with offsets */
    turn_right(40, 0, 0);
    keep_going("turn right", 5000);
    stop_motors();
    sleep_ms(500);

    /* Reverse 5 s */
    move_reverse(40, 0, 0);
    keep_going("reverse", 5000);
    stop_motors();

    sleep_ms(500);
    print_status("finished");
    printf("Done. Press the reset button to run it again.\n");

    while (true)
        sleep_ms(1000); /* stay stopped */
}