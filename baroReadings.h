#ifndef BAR_H
#define BAR_H

#include <Arduino.h>

#include <EnvironmentCalculations.h>
#include <BME280I2C.h>
#include <Wire.h>

#include "limits.h"
#include "types.h"

class baroReadings {

    private:
        // Assumed environmental values:
        float referencePressure = 1018;  // hPa local QFF (official meteor-station reading)
        float outdoorTemp = 20;          // °C  measured local outdoor temp.
        float bar0 = 0;   // meters ... map readings + barometer position
        
        // Default : forced mode, standby time = 1000 ms
        // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off, spi off
        BME280I2C::Settings settings(
            BME280::OSR_X1,
            BME280::OSR_X1,
            BME280::OSR_X1,
            BME280::Mode_Forced,
            BME280::StandbyTime_1000ms,
            BME280::Filter_16,
            BME280::SpiEnable_False,
            BME280I2C::I2CAddr_0x76);

        BME280I2C bme(settings);

    public:

        bool initializeBarometer();
        bar_t *getBarometer();

};