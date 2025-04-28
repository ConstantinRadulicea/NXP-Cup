

#include "features/EDF.h"
#include "Arduino.h"
#include <PWMServo.h>
#include "GlobalVariables.h"

#define EDF_MIN_ACTIVE_TIME_S 0.5f
#define EDF_STEERING_ANGLE_ACTIVATION_RAD radians(12.0f)

volatile PWMServo EDF_motor;
static int8_t local_EDF_active = (int8_t)0;
static float local_EDF_active_remaining_time_s = 0.0f;

void EDF_setup(int edf_pin){
    pinMode(edf_pin, OUTPUT);
    EDF_motor.attach(edf_pin, 1148, 1832);
    EDF_motor.write((int)90);
}

void EDF_activation_loop(float steering_angle_rad){
    if (g_enable_car_engine != 0 && (floatCmp(g_car_speed_mps, 1.0f) >= 0)) {

        if (local_EDF_active != (int8_t)0) {
            local_EDF_active_remaining_time_s -= MillisToSec(g_loop_time_ms);
            if (local_EDF_active_remaining_time_s <= 0.0f) {
                local_EDF_active = (int8_t)0;
            }
        }
        
        if (fabsf(steering_angle_rad) >= fabsf(EDF_STEERING_ANGLE_ACTIVATION_RAD)) {
            local_EDF_active = (int8_t)1;
            local_EDF_active_remaining_time_s = EDF_MIN_ACTIVE_TIME_S;
        }
    }
    else {
        local_EDF_active_remaining_time_s = 0.0f;
        local_EDF_active = (int8_t)0;
    }
    

    if (local_EDF_active != (int8_t)0) {
        EDF_motor.write((int)g_edf_raw_speed);
    }
    else{
        EDF_motor.write((int)90);
    }
}