// Checks if ROBOTSERVO macro is defined
#ifndef ROBOTSERVO_H
// Define ROBOTSERVO macro
#define ROBOTSERVO_H

#include <Arduino.h>
#include "robot_config.h"
#include "current_sensor.h"

/**
  Servo motor class.

  @param pin servo pin number.
  @param lowPulseWidth  pulse width at which robot goes to 0 deg
  @param highPulseWidth pulse width at which robot goes to 180 deg
*/
class RobotServo
{
public:
    // Constructor
    RobotServo(uint8_t pin, uint16_t lowPulseWidth, uint16_t highPulseWidth, CurrentSensor* sensor = nullptr);

    // Member functions
    void setupCurrentSensor(uint8_t pin);
    void setAngle(uint8_t angle);
    uint8_t getAngle() const;

private:
    // private variables
    uint8_t servoPin;
    uint16_t lowServoPulseWidth;
    uint16_t highServoPulseWidth;
    CurrentSensor* currentSensor;
    uint8_t currentAngle = 0;

    // Member function
    void moveServo(uint8_t newAngle);
};

#endif // ROBOTSERVO_H