#include "cloudgripper_servo.h"

RobotServo::RobotServo(uint8_t pin, uint16_t lowPulseWidth, uint16_t highPulseWidth, CurrentSensor* sensor) 
    : servoPin(pin), lowServoPulseWidth(lowPulseWidth), highServoPulseWidth(highPulseWidth), currentSensor(sensor), currentAngle(0){
  pinMode(servoPin, OUTPUT);
  analogWriteFrequency(servoPin, 50); // Change timer to 50 Hz
  analogWriteResolution(12); // PWM value range 0 - 4095
}

void RobotServo::setAngle(uint8_t angle){
  currentAngle = constrain(angle, 0, 180); 
  // calculate the pulse width for the desired position 
  uint32_t pulseWidth = map(currentAngle, 0, 180, lowServoPulseWidth, highServoPulseWidth);
  // convert the pulse width to a duty cycle
  uint32_t duty = (pulseWidth * 4096L) / 20000;
  // send the pulse to the servo
  analogWrite(servoPin, duty);
}

float RobotServo::easeInOutCubic(float t) {
    return t < 0.5f ? 4 * t * t * t : 1 - pow(-2 * t + 2, 3) / 2;
}

void RobotServo::setAngleSmooth(uint8_t targetAngle, uint32_t durationMs) {
    uint32_t startTime = millis();
    uint8_t startAngle = currentAngle;
    int angleDifference = targetAngle - startAngle;
    
    #ifdef DEBUG
    float maxCurrent = 0;  
    #endif

    while (millis() - startTime < durationMs) {
        float t = float(millis() - startTime) / durationMs; // normalized time
        float easingFactor = easeInOutCubic(t); // apply easing function
        uint8_t newAngle = startAngle + easingFactor * angleDifference;
        setAngle(newAngle);

        #ifdef DEBUG
        if (abs(currentSensor->lastMeasuredCurrent) > maxCurrent){
          maxCurrent = abs(currentSensor->lastMeasuredCurrent);
        }    
        #endif

        if (currentSensor && currentSensor->isOverCurrent()) {
            // stop the servo
            #ifdef DEBUG
            Serial.print("Over current value = ");
            Serial.println(currentSensor->lastMeasuredCurrent);
            #endif
            return; 
        }
        delay(1);
    }

    #ifdef DEBUG
    Serial.print("Maximum current in the servo move = ");
    Serial.println(maxCurrent);
    #endif
    setAngle(targetAngle); // ensure the final position is reached
}

uint8_t RobotServo::getAngle() const{
  return currentAngle;
}

