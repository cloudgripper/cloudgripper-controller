// Checks if ROBOT_LIMIT_SWITCH_H macro is defined
#ifndef ROBOT_LIMIT_SWITCH_H
// Define ROBOT_LIMIT_SWITCH_H macro
#define ROBOT_LIMIT_SWITCH_H

#include <Arduino.h>

/**
  Limit Switch class.

  @param switchPin pin number of switch
*/
class LimitSwitch{
    public:
        int switchPin;

        // Constructor
        LimitSwitch(int switchPin){
            pinMode(switchPin, INPUT);
            this->switchPin = switchPin;
        }

        // Member function
        int read(){
            return digitalRead(this->switchPin);
        }
};

#endif // ROBOT_LIMIT_SWITCH_H