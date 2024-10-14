#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "common/pinDef.h"

class Motor {

    public:
        void initialize();
        void driveMotors(float thrust[4]);
        void throttleCut(unsigned long cutOffCmd, float pwm[], bool *armedFly);
};

#endif