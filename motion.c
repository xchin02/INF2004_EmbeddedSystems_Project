/**
 * Robo Pico + Pico W - two-wheel robot movement, all in one file.
 *
 *   Left wheel : Motor 2 (GP10/GP11), encoder on Grove 5 (GP6/GP26)
 *   Right wheel: Motor 1 (GP8/GP9),   encoder on Grove 4 (GP16/GP17)
 *
 * Movement functions:
 *   move_forward(speed, offset_left, offset_right)
 *   move_reverse(speed, offset_left, offset_right)
 *   turn_left (speed, offset_left, offset_right)    keeps turning on the spot
 *   turn_right(speed, offset_left, offset_right)    keeps turning on the spot
 *   spin_left (speed, offset_left, offset_right)    turns exactly 90 deg, stops
 *   spin_right(speed, offset_left, offset_right)    turns exactly 90 deg, stops
 *   stop_motors()
 *   get_speed(), get_distance(), reset_distance(), get_wheel_speeds()
 *
 * speed  : 0..100 (% motor power)
 * offsets: added to that wheel's power (% too). +3 = that wheel gets 3%
 *          more power, -3 = 3% less, whichever way it is spinning.
 *          Use them to make two slightly different motors match:
 *          - going forward and it curves RIGHT -> left wheel is stronger:
 *            give the left a negative offset (or the right a positive one)
 *          - turning and the robot creeps forwards/backwards instead of
 *            spinning on the spot -> one wheel is stronger: lower it.
 *          get_wheel_speeds() shows you both wheels so you can match them.
 */
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

/* ======================= pins =======================
 * Left / right = as seen from behind the robot, facing its front.
 * On this robot the Motor 2 wheel is on the LEFT and the Motor 1 wheel is
 * on the RIGHT. Each encoder goes with its own motor.                   */
#define LEFT_MOTOR_A 10 /* Motor 2: M2A */
#define LEFT_MOTOR_B 11 /* Motor 2: M2B */
#define RIGHT_MOTOR_A 8 /* Motor 1: M1A */
#define RIGHT_MOTOR_B 9 /* Motor 1: M1B */

#define LEFT_ENC_A 6 /* Grove 5 = Motor 2's encoder (GP26 is \
                        shared with Grove 6: keep it free)    */
#define LEFT_ENC_B 26
#define RIGHT_ENC_A 16 /* Grove 4 = Motor 1's encoder           */
#define RIGHT_ENC_B 17

/* Set to 1 if a wheel spins backwards on move_forward(),
 * or if its speed shows NEGATIVE when it rolls forwards.
 * The two motors face opposite ways, so one side is flipped. A motor and
 * its own encoder flip together.                                         */
#define LEFT_MOTOR_INVERT 0
#define RIGHT_MOTOR_INVERT 1
#define LEFT_ENC_INVERT 0
#define RIGHT_ENC_INVERT 1

/* ================= robot measurements ================= */
#define COUNTS_PER_REV (11 * 4 * 30) /* encoder PPR x 4 x gear ratio  */
#define WHEEL_DIAMETER_MM 65.0f
#define TRACK_WIDTH_MM 150.0f /* centre of left tyre to centre \
                                 of right tyre                */
#define MM_PER_COUNT (3.14159265f * WHEEL_DIAMETER_MM / COUNTS_PER_REV)

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

/* speed: -100 (full backwards) .. 0 (brake) .. +100 (full forwards) */
static void set_motor(uint pin_a, uint pin_b, int speed, bool invert)
{
    if (invert)
        speed = -speed;
    if (speed > 100)
        speed = 100;
    if (speed < -100)
        speed = -100;

    uint16_t level = (uint16_t)(abs(speed) * (pwm_top + 1) / 100);

    if (speed > 0)
    {
        pwm_set_gpio_level(pin_a, level);
        pwm_set_gpio_level(pin_b, 0);
    }
    else if (speed < 0)
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

static void set_wheels(int left, int right)
{
    set_motor(LEFT_MOTOR_A, LEFT_MOTOR_B, left, LEFT_MOTOR_INVERT);
    set_motor(RIGHT_MOTOR_A, RIGHT_MOTOR_B, right, RIGHT_MOTOR_INVERT);
}

/* speed + offset, kept between 0 and 100 */
static int with_offset(int speed, int offset)
{
    int s = speed + offset;
    if (s < 0)
        s = 0;
    if (s > 100)
        s = 100;
    return s;
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
static volatile float left_speed, right_speed; /* mm/s */
static struct repeating_timer speed_timer;

static bool speed_timer_cb(struct repeating_timer *t)
{
    (void)t;
    static int32_t last_l, last_r;
    int32_t l = left_counts(), r = right_counts();
    left_speed = (float)(l - last_l) * MM_PER_COUNT * (1000.0f / SPEED_SAMPLE_MS);
    right_speed = (float)(r - last_r) * MM_PER_COUNT * (1000.0f / SPEED_SAMPLE_MS);
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

/* robot speed in mm/s: + forwards, - backwards */
float get_speed(void)
{
    return 0.5f * (left_speed + right_speed);
}

/* each wheel's speed in mm/s - handy for tuning offsets */
void get_wheel_speeds(float *left_mm_s, float *right_mm_s)
{
    *left_mm_s = left_speed;
    *right_mm_s = right_speed;
}

/* distance in mm since reset_distance(): + forwards, - backwards */
float get_distance(void)
{
    int32_t dl = left_counts() - dist_start_l;
    int32_t dr = right_counts() - dist_start_r;
    return 0.5f * (float)(dl + dr) * MM_PER_COUNT;
}

void reset_distance(void)
{
    dist_start_l = left_counts();
    dist_start_r = right_counts();
}

/* =====================================================================
 *  Movement
 * ===================================================================== */
void stop_motors(void)
{
    set_wheels(0, 0); /* brake */
}

/* Both wheels forwards. Keeps going until stop_motors() or another move. */
void move_forward(int speed, int offset_left, int offset_right)
{
    set_wheels(with_offset(speed, offset_left), with_offset(speed, offset_right));
}

/* Both wheels backwards. Keeps going until stop_motors() or another move. */
void move_reverse(int speed, int offset_left, int offset_right)
{
    set_wheels(-with_offset(speed, offset_left), -with_offset(speed, offset_right));
}

/* Turn left on the spot (left wheel back, right wheel forward) using the
 * offsets. Keeps turning until stop_motors() or another move. */
void turn_left(int speed, int offset_left, int offset_right)
{
    set_wheels(-with_offset(speed, offset_left), with_offset(speed, offset_right));
}

/* Turn right on the spot (left wheel forward, right wheel back) using the
 * offsets. Keeps turning until stop_motors() or another move. */
void turn_right(int speed, int offset_left, int offset_right)
{
    set_wheels(with_offset(speed, offset_left), -with_offset(speed, offset_right));
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

        float turned = wheel_counts * MM_PER_COUNT / (TRACK_WIDTH_MM * TURN_CALIBRATION) * RAD_TO_DEG;
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

        int s = speed;
        if (remaining < TURN_SLOW_DEG && s > TURN_SLOW_SPEED)
            s = TURN_SLOW_SPEED; /* slow down near the end = more exact */

        int l = with_offset(s, offset_left);
        int r = with_offset(s, offset_right);
        set_wheels(-dir * l, dir * r); /* left turn: left back, right forward */
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
    printf("%-12s speed=%6.1f mm/s  distance=%7.1f mm   (left %6.1f / right %6.1f mm/s)\n",
           label, get_speed(), get_distance(), l, r);
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

    sleep_ms(3000); /* time to open the serial monitor */

#if ENCODER_CHECK
    printf("\nENCODER CHECK: roll each wheel forward one full turn by hand.\n");
    printf("Each count should go UP by about %d.\n", COUNTS_PER_REV);
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