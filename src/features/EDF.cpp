

#include "features/EDF.h"
#include "Arduino.h"
#include <PWMServo.h>

volatile PWMServo EDF_motor;

void EDF_setup(int edf_pin){
    pinMode(edf_pin, OUTPUT);
    EDF_motor.attach(edf_pin, 1148, 1832);
    EDF_motor.write(90);
}