/*
 * ir-line-sensor.h
 * Left and right MH-sensors that straddle the guide line (Buddy 3).
 */

#ifndef IR_LINE_SENSOR_H
#define IR_LINE_SENSOR_H

#include <stdbool.h>

// Pins from the Design Review pin table (Section 2).
#define IR_LEFT_PIN 13  // MH-sensor #1 DO (Grove 1)
#define IR_RIGHT_PIN 14 // MH-sensor #2 DO (Grove 6)

// 1 = DO reads HIGH over the black line (usual for MH-sensors).
// Set to 0 if the serial output shows the opposite.
#define IR_LINE_IS_HIGH 1

void ir_line_sensor_init(void);
bool ir_line_is_left_on_line(void);
bool ir_line_is_right_on_line(void);
int ir_line_left_raw(void); // raw DO level 0/1, for checking polarity
int ir_line_right_raw(void);

#endif // IR_LINE_SENSOR_H