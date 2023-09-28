#ifndef CURRENT_SENSOR
#define CURRENT_SENSOR

#include <Arduino.h>
#include "ADC.h"

class CurrentSensor {
    private:
        uint8_t pin;
        ADC* adc;

    public:
        float lastMeasuredCurrent = 0;

        CurrentSensor(uint8_t pin) : pin(pin) , adc(new ADC()){
            adc->adc0->setAveraging(5);
            adc->adc0->setResolution(12);
            adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
            adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

            adc->adc1->setAveraging(5);
            adc->adc1->setResolution(12);
            adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
            adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
        
            pinMode(pin, INPUT);
        }

        float readCurrent() {
            float adcReading = adc->analogRead(pin);
            float voltage = (adcReading - 2067) * (3.3 / 4096.0) * (2.0 / 3.0); 
            float current = voltage / 0.185;

            #ifdef DEBUG
            Serial.println("Current Sensor Reading");
            Serial.print("adcReading = ");
            Serial.println(adcReading);
            Serial.print("voltage = ");
            Serial.println(voltage);
            Serial.print("current = ");
            Serial.println(current);
            #endif

            lastMeasuredCurrent = current;
            return current;
        }

        bool isOverCurrent () {
            return abs(readCurrent()) > CURRENT_THRESHOLD;
}
};

#endif // CURRENT_SENSOR