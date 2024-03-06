#ifndef CURRENT_SENSOR_REF
#define CURRENT_SENSOR_REF

#include <Arduino.h>
#include <ADC.h>
#include "robot_config.h"

class CurrentSensorReference {
private:
    ADC* adc;
    uint8_t ref_voltage_pin;

    CurrentSensorReference() : adc(new ADC()), ref_voltage_pin(PIN_CURRENT_REFERENCE) {
        adc->adc0->setAveraging(5);
        adc->adc0->setResolution(12);
        adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
        adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

        adc->adc1->setAveraging(5);
        adc->adc1->setResolution(12);
        adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
        adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
        
        pinMode(ref_voltage_pin, INPUT);
    }

    CurrentSensorReference(const CurrentSensorReference&) = delete;
    CurrentSensorReference& operator=(const CurrentSensorReference&) = delete;

public:
    // Singleton get instance method
    static CurrentSensorReference& getInstance() {
        static CurrentSensorReference instance;
        return instance;
    }

    float getReferenceVoltage() {
        int refReading = adc->analogRead(ref_voltage_pin);
        float refVoltage = (refReading) * (3.3 / 4096.0) * (3.0 / 2.0);
        return refVoltage;
    }
};

#endif // CURRENT_SENSOR_REF