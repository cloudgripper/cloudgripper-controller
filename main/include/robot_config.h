// Checks if ROBOT_CONFIG_H macro is defined
#ifndef ROBOT_CONFIG_H
// Define ROBOT_CONFIG_H macro
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include "servo_config.h"

#define DEBUG

// Stepper motor pins
constexpr int ENABLE_PIN = 8;
constexpr int DIR_PIN_RIGHT_STEPPER = 5;
constexpr int DIR_PIN_LEFT_STEPPER = 6;
constexpr int STEP_PIN_RIGHT_STEPPER = 2;
constexpr int STEP_PIN_LEFT_STEPPER = 3;

// Motor wiring
constexpr bool IS_MOTOR_RIGHT_WIRING_REVERSED = true;
constexpr bool IS_MOTOR_LEFT_WIRING_REVERSED = false;

// Stepper Encoder pins
constexpr int A_CHANNEL_RIGHT_STEPPER_ENCODER = 28;
constexpr int B_CHANNEL_RIGHT_STEPPER_ENCODER = 29;
constexpr int A_CHANNEL_LEFT_STEPPER_ENCODER = 30;
constexpr int B_CHANNEL_LEFT_STEPPER_ENCODER = 31;

// Limit switch pins
constexpr int PIN_X_SWITCH = 7;
constexpr int PIN_Y_SWITCH = 4;

// Servo motor pins
constexpr int PIN_Z_AXIS_SERVO = 9;
constexpr int PIN_CLAW_SERVO = 8;
constexpr int PIN_ROTATION_SERVO = 10;

// Servo Limits
constexpr int SERVO_Z_LOW_LIMIT = 0;
constexpr int SERVO_Z_HIGH_LIMIT = 180;
constexpr int SERVO_ROT_LOW_LIMIT = 0;
constexpr int SERVO_ROT_HIGH_LIMIT = 180;
constexpr int SERVO_GRIP_LOW_LIMIT = 0;
constexpr int SERVO_GRIP_HIGH_LIMIT = 90;

// Servo current sensor pin and threshold
constexpr int PIN_CURRENT_SENSOR_GRIP = 14; 
constexpr int PIN_CURRENT_SENSOR_Z_AXIS = 15; 
constexpr int PIN_CURRENT_SENSOR_ROTATION = 16;
constexpr float CURRENT_THRESHOLD = 1.0;

// Motor parameters
constexpr int ENCODER_PULSES_PER_REVOLUTION = 4000;     // 4000 encoder_pulses / rev
constexpr int STEPS_PER_REVOLITION = 200 * 16;          // 200 steps/rev * 16x microstepping
constexpr int STEPS_PER_MM = STEPS_PER_REVOLITION / 40; // 40mm/rev
constexpr float MM_PER_ENCODER_PULSES = 40.0f / ENCODER_PULSES_PER_REVOLUTION;
constexpr int PULSE_WIDTH = 150; // was 100, changed to remove jerk   //step pulse-width (uS)

// Switch definition
constexpr bool NORMAL_SWITCH_STATE = 0;

// Plotter definitions
constexpr int BAUD_RATE = 9600;
constexpr int STRING_SIZE = 256;
extern char BUFFER[STRING_SIZE + 1];

// Gcode definitions
extern unsigned int INDEX; // buffer index
extern int START, FINISH;
extern float X, Y; // gcode float values held here

// Flags
extern bool CW;  // Does not affect motor direction
extern bool CCW; // Does not affect motor direction

// Variables
extern bool DIRECTION1; // motor directions can be changed in step_motors()
extern bool DIRECTION2;

#endif // ROBOT_CONFIG_H