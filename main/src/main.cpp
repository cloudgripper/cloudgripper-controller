/*
 * Name: cloudgripper-controller
 * Purpose: Low level control of CloudGripper
 *
 * This code provides low-level control of CloudGripper, using the Arduino platform. It controls
 * the robot's servo motors, stepper motors, and limit switches to position the gripper.
 *
 * @author Zahid Muhammad
 * @version 2.0
 */

#include <Arduino.h>
#include <teensystep4.h>
#include <Encoder.h>
#include "robot_config.h"
#include "cloudgripper_servo.h"
#include "cloudgripper_limit_switch.h"
#include "process_serial_input.h"
#include "cloudgripper.h"
#include "robot_command.h"
#include "current_sensor.h"
#include "servo_config.h"

// CoreXY variables
// XY robot position (integer steps)
long newRobotX = 0;
long newRobotY = 0;
long robotX = 0;
long robotY = 0;

long leftEncoderPosition = -999;
long rightEncoderPosition = -999;

/*
  Innitialize Sensors
*/
// Stepper Encoder objects
Encoder rightStepperEncoder(A_CHANNEL_RIGHT_STEPPER_ENCODER, B_CHANNEL_RIGHT_STEPPER_ENCODER);
Encoder leftStepperEncoder(A_CHANNEL_LEFT_STEPPER_ENCODER, B_CHANNEL_LEFT_STEPPER_ENCODER);

// IR Limit Switch objects
LimitSwitch xLimitSwitch(PIN_X_SWITCH);
LimitSwitch yLimitSwitch(PIN_Y_SWITCH);

// Current Sensor pointers
CurrentSensor *sensor_claw = new CurrentSensor(PIN_CURRENT_SENSOR_GRIP);
CurrentSensor *sensor_z_axis = new CurrentSensor(PIN_CURRENT_SENSOR_Z_AXIS);
CurrentSensor *sensor_rotation = new CurrentSensor(PIN_CURRENT_SENSOR_ROTATION);
CurrentSensor* sensor_ref = new CurrentSensor(PIN_CURRENT_REFERENCE);

/*
  Innitialize Actuators
*/
// Servo objects
RobotServo clawServo(PIN_CLAW_SERVO, SERVO_CLAW_LOW_PULSE_WIDTH, SERVO_CLAW_HIGH_PULSE_WIDTH, sensor_claw);
RobotServo zaxisServo(PIN_Z_AXIS_SERVO, SERVO_Z_LOW_PULSE_WIDTH, SERVO_Z_HIGH_PULSE_WIDTH, sensor_z_axis);
RobotServo rotationServo(PIN_ROTATION_SERVO, SERVO_ROT_LOW_PULSE_WIDTH, SERVO_ROT_HIGH_PULSE_WIDTH, sensor_rotation);

// Stepper Motor objects
TS4::Stepper rightStepper(STEP_PIN_RIGHT_STEPPER, DIR_PIN_RIGHT_STEPPER);
TS4::Stepper leftStepper(STEP_PIN_LEFT_STEPPER, DIR_PIN_LEFT_STEPPER);
TS4::StepperGroup stepperGroup{rightStepper, leftStepper};

/*
  Robot object
*/
Robot robot(clawServo, zaxisServo, rotationServo, rightStepper, leftStepper, stepperGroup, rightStepperEncoder, leftStepperEncoder, xLimitSwitch, yLimitSwitch);

void serialFlush();

void setup()
{

  // Set pin mode
  pinMode(ENABLE_PIN, OUTPUT);

  // Innitialize TeensyStepper4
  TS4::begin();

  // Set stepper speed and acceleration
  rightStepper
      .setMaxSpeed(10'000)
      .setAcceleration(30'000);

  leftStepper
      .setMaxSpeed(10'000)
      .setAcceleration(30'000);

  // Set initial stpper pin states
  digitalWrite(DIR_PIN_RIGHT_STEPPER, CCW);
  digitalWrite(DIR_PIN_LEFT_STEPPER, CCW);
  digitalWrite(STEP_PIN_RIGHT_STEPPER, LOW);
  digitalWrite(STEP_PIN_LEFT_STEPPER, LOW);
  digitalWrite(ENABLE_PIN, LOW);

  // Set stepper initial position
  rightStepper.setPosition(0);
  leftStepper.setPosition(0);

  // Plotter setup
  memset(BUFFER, '\0', sizeof(BUFFER)); // fill with string terminators

  // Establish serial link
  Serial.begin(BAUD_RATE);

  // Flush the buffers
  Serial.flush(); // clear TX buffer
  serialFlush();  // clear RX buffer

  // Display commands
  menu();
}

String handleSerialCommands();
void executeRobotCommand(RobotCommand command, Robot &robot);

void loop()
{
  // Read serial communication
  String robotCommandString = handleSerialCommands();

  if (robotCommandString.length() > 0)
  {
    // Parse robot command
    RobotCommand command = parseRobotCommand(robotCommandString);

    // Execute robot command
    executeRobotCommand(command, robot);
  }
}

void executeRobotCommand(RobotCommand command, Robot &robot)
{
  switch (command.type)
  {
  case RobotCommand::MOVE_TO:
    robot.moveTo(command.x, command.y);
    break;

  case RobotCommand::ROTATE_GRIPPER:
    robot.rotateGripper(command.angle);
    break;

  case RobotCommand::OPEN_CLOSE_GRIPPER:
    robot.openCloseGripper(command.angle);
    break;

  case RobotCommand::UP_DOWN_GRIPPER:
    robot.upDownGripper(command.angle);
    break;

  case RobotCommand::CALIBRATE:
    robot.calibrate();
    break;

  case RobotCommand::STEP:
    robot.stepRobotPosition(command.direction, command.stepDistance);
    break;
  }
}

String handleSerialCommands()
{
  unsigned int bytecount = 0;
  char inputByte;
  String robotCommand = "";

  while (Serial.available() && bytecount < 10)
  {
    inputByte = (char)Serial.read();
    BUFFER[INDEX++] = inputByte;

    if (inputByte == '\n' && INDEX > 1)
    {
      robotCommand = BUFFER;

      // Clear the buffer with null-terminators
      memset(BUFFER, '\0', sizeof(BUFFER));
      INDEX = 0;

      return robotCommand;
    }

    // Prevent buffer overflow
    if (INDEX >= sizeof(BUFFER))
    {
      INDEX = 0;
    }

    bytecount++;
  }

  return "";
}

void serialFlush()
{
  while (Serial.available() > 0)
  {
    Serial.read();
  }
}