// wheel.h - motor + encoder driver for RP2040 (Pico W) on the RoboPico board
#ifndef WHEEL_H
#define WHEEL_H

#include <stdint.h>

// ---------------- Pins ----------------
// If a wheel spins the wrong way, swap its _A and _B motor pins.
#define LEFT_MOTOR_A 11
#define LEFT_MOTOR_B 10
#define RIGHT_MOTOR_A 8
#define RIGHT_MOTOR_B 9

// If an encoder counts negative while driving forward, swap its _A and _B pins.
#define LEFT_ENC_A 16
#define LEFT_ENC_B 17
#define RIGHT_ENC_A 26
#define RIGHT_ENC_B 6

// ---------------- Speeds (percent, 0-100) ----------------
#define BASE_SPEED 60 // moveForward() / reverse() speed before offsets
#define TURN_SPEED 45 // speed of both wheels while turning
// Turn timing (TURN_90_MS) is set in motion.c, next to the turn functions.

// ---------------- Functions ----------------
void wheel_init(void); // call once at start-up

// Start driving and return immediately (use sleep_ms() then stopMotors()).
// Each offset is added to BASE_SPEED for that wheel, e.g.
//   moveForward(10, 0)  -> left 70%, right 60% -> gentle curve to the right
//   moveForward(0, 10)  -> curve to the left
void moveForward(int leftOffset, int rightOffset);
void reverse(int leftOffset, int rightOffset);

// Spin on the spot by the given number of degrees, e.g. turn_left(90), turn_right(45).
// Each call turns once and only returns when the turn is finished and the
// robot has stopped, so the next line of your code waits for it.
void turn_left(int degrees);  // left wheel backward, right wheel forward
void turn_right(int degrees); // left wheel forward, right wheel backward

void stopMotors(void); // active brake on both wheels

int32_t getLeftCount(void);
int32_t getRightCount(void);
void resetEncoders(void);

void encoderCheck(void); // calibration: motors off, prints encoder counts forever

#endif