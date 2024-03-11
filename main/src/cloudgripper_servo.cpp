#include "cloudgripper_servo.h"

RobotServo::RobotServo(uint8_t pin, uint16_t lowPulseWidth, uint16_t highPulseWidth, CurrentSensor* sensor) 
    : servoPin(pin), lowServoPulseWidth(lowPulseWidth), highServoPulseWidth(highPulseWidth), currentSensor(sensor), currentAngle(0){
  pinMode(servoPin, OUTPUT);
}

void RobotServo::setAngle(uint8_t angle){
  uint8_t currentAngle = constrain(angle, 0, 180); 
  moveServo(currentAngle);
  delay(5);

  this->currentAngle = currentAngle;
}

void RobotServo::moveServo(uint8_t newAngle)
{
  // calculate the pulse width for the desired position
  uint32_t pulseWidth = map(newAngle, 0, 180, lowServoPulseWidth, highServoPulseWidth);

  // send the pulse to the servo
  for (uint32_t i =0; i < 20; i++){
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(20000 - pulseWidth);
  }
}

uint8_t RobotServo::getAngle() const{
  return this->currentAngle;
}

float RobotServo::getCurrent() const
{
  return currentSensor->readCurrentAvg();
}

