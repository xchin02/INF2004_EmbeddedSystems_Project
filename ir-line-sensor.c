/*
 * line_edge_test.c
 * Bench test: do the left and right MH-sensors see the edges of the line?
 * Polls both digital outputs every 10 ms (same rate as the line task)
 * and prints a message whenever either sensor changes state.
 */

#include <stdio.h>
#include "ir-line-sensor.h"
#include "pico/stdlib.h"

static void ir_pin_init(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin); // the MH-sensor board drives DO itself
}

static bool level_is_line(int level)
{
    return IR_LINE_IS_HIGH ? (level != 0) : (level == 0);
}

void ir_line_sensor_init(void)
{
    ir_pin_init(IR_LEFT_PIN);
    ir_pin_init(IR_RIGHT_PIN);
}

int ir_line_left_raw(void)
{
    return gpio_get(IR_LEFT_PIN) ? 1 : 0;
}

int ir_line_right_raw(void)
{
    return gpio_get(IR_RIGHT_PIN) ? 1 : 0;
}

bool ir_line_is_left_on_line(void)
{
    return level_is_line(ir_line_left_raw());
}

bool ir_line_is_right_on_line(void)
{
    return level_is_line(ir_line_right_raw());
}

// static void ir_init(uint pin)
// {
//     gpio_init(pin);
//     gpio_set_dir(pin, GPIO_IN);
// }

// // Returns 1 when the sensor is over the line, 0 when over the floor.
// static int on_line(uint pin)
// {
//     int level = gpio_get(pin);
//     return LINE_IS_HIGH ? level : !level;
// }

// static void report(const char *name, int now_on_line, uint32_t t_ms, uint32_t count)
// {
//     printf("[%8lu ms] %-5s sensor: %s  (change #%lu)\n",
//            (unsigned long)t_ms, name,
//            now_on_line ? "floor -> LINE (line edge entered)"
//                        : "LINE -> floor (line edge left)   ",
//            (unsigned long)count);
// }

// int main(void)
// {
//     stdio_init_all();
//     sleep_ms(2000);   // give the serial monitor time to connect

//     ir_init(LEFT_IR_PIN);
//     ir_init(RIGHT_IR_PIN);

//     printf("\nline_edge_test started. Left=GP%d  Right=GP%d  line is %s\n",
//            LEFT_IR_PIN, RIGHT_IR_PIN, LINE_IS_HIGH ? "HIGH" : "LOW");
//     printf("Slide each sensor over the line edge and watch for a change.\n\n");

//     int prev_l = on_line(LEFT_IR_PIN);
//     int prev_r = on_line(RIGHT_IR_PIN);
//     uint32_t count_l = 0, count_r = 0;

//     printf("Start state: left=%s right=%s\n\n",
//            prev_l ? "LINE" : "floor", prev_r ? "LINE" : "floor");

//     while (true) {
//         int l = on_line(LEFT_IR_PIN);
//         int r = on_line(RIGHT_IR_PIN);
//         uint32_t t_ms = to_ms_since_boot(get_absolute_time());

//         if (l != prev_l) {
//             report("LEFT", l, t_ms, ++count_l);
//             prev_l = l;
//         }
//         if (r != prev_r) {
//             report("RIGHT", r, t_ms, ++count_r);
//             prev_r = r;
//         }

//         sleep_ms(SAMPLE_MS);
//     }
// }