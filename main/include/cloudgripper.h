// Checks if ROBOT_H macro is defined
#ifndef ROBOT_H
// Define ROBOT_H macro
#define ROBOT_H

#include <Arduino.h>
#include <teensystep4.h>
#include <Encoder.h>
#include "robot_config.h"
#include "cloudgripper_servo.h"
#include "cloudgripper_limit_switch.h"

/*
  Cloudgripper robot class
*/
class Robot
{
public:
    // Constructor
    Robot(RobotServo &clawServo, RobotServo &zaxisServo, RobotServo &rotationServo,
          TS4::Stepper &rightStepper, TS4::Stepper &leftStepper, TS4::StepperGroup &stepperGroup,
          Encoder &rightStepperEncoder, Encoder &leftStepperEncoder,
          LimitSwitch &xLimitSwitch, LimitSwitch &yLimitSwitch);

    // Member functions
    void moveTo(float x, float y);
    void stepRobotPosition(String direction, float stepDistance);
    void rotateGripper(int angle);
    void openCloseGripper(int angle);
    void upDownGripper(int angle);
    void printState();
    void calibrate();

private:
    RobotServo &clawServo;
    RobotServo &zaxisServo;
    RobotServo &rotationServo;
    TS4::Stepper &rightStepper;
    TS4::Stepper &leftStepper;
    TS4::StepperGroup &stepperGroup;
    Encoder &rightStepperEncoder;
    Encoder &leftStepperEncoder;
    LimitSwitch &xLimitSwitch;
    LimitSwitch &yLimitSwitch;

    void fixPositionUsingEncoder();
    void stepMotors();
    bool motor1RotationDirection;
    bool motor2RotationDirection;
    long robotXposition;
    long robotYposition;
};

#endif // ROBOT_H
