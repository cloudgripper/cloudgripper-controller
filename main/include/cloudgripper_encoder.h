// Checks if ROBOT_ENCODER_H macro is defined
#ifndef ROBOT_ENCODER_H
// Define ROBOT_ENCODER_H macro
#define ROBOT_ENCODER_H_H

#include <Encoder.h>

/**
  Stepper Encoder class.

  @param pinAChannel pin number of A channel
  @param pinBChannel pin number of A channel
*/
class StepperEncoder{
    private:
        Encoder encoder;
        int position;
        int pinAChannel;
        int pinBChannel;

    public:
        // Constructor
        StepperEncoder(int pinAChannel, int pinBChannel): encoder(pinAChannel, pinBChannel){
            this->pinAChannel = pinAChannel;
            this->pinBChannel = pinBChannel;
        }

        // Member functions
        int read(){
            this->position = encoder.read();
            return this->position;
        }
        void write(int val){
            encoder.write(val);
            this->position = val;
        }   
};

#endif // ROBOT_ENCODER_H_H