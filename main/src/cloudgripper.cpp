#include "cloudgripper.h"
#include "robot_config.h"

Robot::Robot(RobotServo &clawServo, RobotServo &zaxisServo, RobotServo &rotationServo,
             TS4::Stepper &rightStepper, TS4::Stepper &leftStepper, TS4::StepperGroup &stepperGroup,
             Encoder &rightStepperEncoder, Encoder &leftStepperEncoder,
             LimitSwitch &xLimitSwitch, LimitSwitch &yLimitSwitch)
    : clawServo(clawServo), zaxisServo(zaxisServo), rotationServo(rotationServo),
      rightStepper(rightStepper), leftStepper(leftStepper), stepperGroup(stepperGroup),
      rightStepperEncoder(rightStepperEncoder), leftStepperEncoder(leftStepperEncoder),
      xLimitSwitch(xLimitSwitch), yLimitSwitch(yLimitSwitch) {}

void Robot::moveTo(float x, float y)
{
    // Convert target (x,y) position to steps
    long newRobotX = round(x * STEPS_PER_MM);
    long newRobotY = round(y * STEPS_PER_MM);

    // Calculate target motor steps
    int motorRightSteps;
    int motorLeftSteps;
    motorRightSteps = (newRobotX - newRobotY);
    motorLeftSteps = (newRobotX + newRobotY);

    // Calculate target encoder position
    long goalPosL = int((float(motorLeftSteps) / float(STEPS_PER_REVOLUTION)) * float(ENCODER_PULSES_PER_REVOLUTION));
    long goalPosR = int((float(motorRightSteps) / float(STEPS_PER_REVOLUTION)) * float(ENCODER_PULSES_PER_REVOLUTION));

    // Move motors to target position
    rightStepper.setTargetAbs((IS_MOTOR_RIGHT_WIRING_REVERSED ? -1 : 1) * motorRightSteps);
    leftStepper.setTargetAbs((IS_MOTOR_LEFT_WIRING_REVERSED ? -1 : 1) * motorLeftSteps);
    stepperGroup.move();
    delay(5);

    // Remember last co-ordinate
    // this->robotXposition = newRobotX;
    // this->robotYposition = newRobotY;
    this->robotXposition = x;
    this->robotYposition = y;
}

void Robot::stepRobotPosition(String direction, float stepDistance = 1)
{
    // Increase the acceleration of the stepper motors to step quickly
    rightStepper.setAcceleration(50'000);
    leftStepper.setAcceleration(50'000);

    // Step to the correct direction using moveTo function
    if (direction == "right")
    {
        moveTo(this->robotXposition + stepDistance, this->robotYposition);
    }
    if (direction == "left")
    {
        moveTo(this->robotXposition - stepDistance, this->robotYposition);
    }
    if (direction == "forward")
    {
        moveTo(this->robotXposition, this->robotYposition + stepDistance);
    }
    if (direction == "backward")
    {
        moveTo(this->robotXposition, this->robotYposition - stepDistance);
    }

    // Set the acceleration back to the original value
    rightStepper.setAcceleration(30'000);
    leftStepper.setAcceleration(30'000);
}

void Robot::fixPositionUsingEncoder()
{
    // Set stepper position to match the encoder position
    long leftEncoderPosition = -1 * leftStepperEncoder.read();
    long rightEncoderPosition = rightStepperEncoder.read();
    int stepsToSetLeftMotor = int((float(leftEncoderPosition) / float(ENCODER_PULSES_PER_REVOLUTION)) * float(STEPS_PER_REVOLUTION));
    int stepsToSetRightMotor = int((float(rightEncoderPosition) / float(ENCODER_PULSES_PER_REVOLUTION)) * float(STEPS_PER_REVOLUTION));

    leftStepper.setPosition(stepsToSetLeftMotor);
    rightStepper.setPosition(stepsToSetRightMotor);
}
void Robot::stepMotors()
{
    // Use manual pin control to step both the stepper motors
    digitalWriteFast(DIR_PIN_RIGHT_STEPPER, DIRECTION1);
    digitalWriteFast(DIR_PIN_LEFT_STEPPER, DIRECTION2);

    digitalWriteFast(STEP_PIN_RIGHT_STEPPER, LOW);
    digitalWriteFast(STEP_PIN_LEFT_STEPPER, LOW);
    delayMicroseconds(PULSE_WIDTH);
    digitalWriteFast(STEP_PIN_RIGHT_STEPPER, HIGH);
    digitalWriteFast(STEP_PIN_LEFT_STEPPER, HIGH);
    delayMicroseconds(PULSE_WIDTH);
}

// Function to set the Gripper rotation
void Robot::rotateGripper(int angle) { rotationServo.setAngle(angle); }
// Function to set the Gripper claw position
void Robot::openCloseGripper(int angle) { clawServo.setAngle(angle); }
// Function to set the Gripper Z-axis position
void Robot::upDownGripper(int angle) { zaxisServo.setAngle(angle); }

void Robot::printState() {
    
    // Read new encoder position
    long rightEncoderPosition =  (IS_ENCODER_RIGHT_WIRING_REVERSED  ? -1 : 1) * rightStepperEncoder.read();
    long leftEncoderPosition =  (IS_ENCODER_LEFT_WIRING_REVERSED ? -1 : 1) * leftStepperEncoder.read();

    // Calculate stepper position corresponding to new encoder position
    long rightStepsMeasured  = long((float(rightEncoderPosition) / float(ENCODER_PULSES_PER_REVOLUTION)) * float(STEPS_PER_REVOLUTION));
    long leftStepsMeasured  = long((float(leftEncoderPosition) / float(ENCODER_PULSES_PER_REVOLUTION)) * float(STEPS_PER_REVOLUTION));
    
    long newRobotXMeasured  = (leftStepsMeasured + rightStepsMeasured) / 2;
    long newRobotYMeasured  = (leftStepsMeasured - rightStepsMeasured) / 2;

    float xMeasured = newRobotXMeasured  / (float)STEPS_PER_MM;
    float yMeasured = newRobotYMeasured  / (float)STEPS_PER_MM;

    // Send over serial in a human-readable format
    // x, y, z_angle, rotation_angle, claw_angle, z_current, rotation_current, claw_current, rotation_current
    Serial.print("STATE ");
    Serial.print(xMeasured);
    Serial.print(" ");
    Serial.print(yMeasured);
    Serial.print(" ");
    Serial.print(zaxisServo.getAngle());
    Serial.print(" ");
    Serial.print(rotationServo.getAngle());
    Serial.print(" ");
    Serial.print(clawServo.getAngle());
    Serial.print(" ");
    Serial.print(zaxisServo.getCurrent());
    Serial.print(" ");
    Serial.print(rotationServo.getCurrent());
    Serial.print(" ");
    Serial.println(clawServo.getCurrent());
}

void Robot::calibrate()
{
    // Set gripper to certain state before calibrating
    rotateGripper(0);
    openCloseGripper(0);
    upDownGripper(0);

    // Move right till calibration switch is pressed
    while (xLimitSwitch.read() == NORMAL_SWITCH_STATE)
    {
        stepRobotPosition("right");
        while(rightStepper.isMoving || leftStepper.isMoving){
            delay(2);
        }
    }
    // Move backwards till calibration switch is pressed
    while (yLimitSwitch.read() == NORMAL_SWITCH_STATE)
    {
        stepRobotPosition("backward");
        while(rightStepper.isMoving || leftStepper.isMoving){
            delay(2);
        }
    }
    // Set the robot position to (0,0)
    this->robotXposition = 0;
    this->robotYposition = 0;
    // Set the stepper motor position to zero
    rightStepper.setPosition(0);
    leftStepper.setPosition(0);
    // Set encoder position to zero
    leftStepperEncoder.write(0);
    rightStepperEncoder.write(0);
    delay(200);
}