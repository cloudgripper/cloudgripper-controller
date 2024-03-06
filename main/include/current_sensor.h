#ifndef CURRENT_SENSOR
#define CURRENT_SENSOR

#include <Arduino.h>
#include <ADC.h>
#include "current_ref.h"

class CurrentSensor {
    private:
        uint8_t pin;
        ADC* adc;
        const int numSamples = 20;

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
            const float adcResolution = 4096.0;
            const float adcRefVoltage = 3.3;
            const float voltageDividerFactor = 3.0 / 2.0;
            const float acsSensitivity_5V = 0.185f; // ACS712 sensitivity at 5V supply

            float adcReading = adc->analogRead(pin);
            float supplyVoltage = CurrentSensorReference::getInstance().getReferenceVoltage();
            float voltageOutput = (adcReading) * (adcRefVoltage / adcResolution) * voltageDividerFactor; 

            float zeroCurrentOutput = supplyVoltage / 2.0f;  // Zero current output voltage scales with supply voltage
            float sensitivity = acsSensitivity_5V * (supplyVoltage / 5.0f);  // Sensitivity scales with supply voltage

            float current = (voltageOutput - zeroCurrentOutput) / sensitivity;

            #ifdef DEBUG
            Serial.println("Current Sensor Reading");
            Serial.print("adcReading = ");
            Serial.println(adcReading);
            Serial.print("voltage = ");
            Serial.println(voltageOutput);
            Serial.print("current = ");
            Serial.println(current);
            #endif

            return current;
        }

        float readCurrentAvg() {
            float currentSum = 0;
            float average = 0;
            for (int i = 0; i < numSamples; i++) {
                currentSum += readCurrent();
            }
            average = currentSum / numSamples;
            // Serial.println(average);
            lastMeasuredCurrent = average;
            return average;
        }
};

#endif // CURRENT_SENSOR