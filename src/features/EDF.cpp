

#include "features/EDF.h"
#include "Arduino.h"
#include <PWMServo.h>
#include "GlobalVariables.h"

#define EDF_MIN_ACTIVE_TIME_S 0.5f
#define EDF_STEERING_ANGLE_ACTIVATION_RAD radians(12.0f)
#define EDF_MIN_VEHICLE_SPEED_MPS 0.1f
#define EDF_IDLE_RAW_SPEED 100
#define EDF_STANDSTILL_RAW_SPEED 90

volatile PWMServo EDF_motor;
static int8_t local_EDF_active = (int8_t)0;
static float local_EDF_active_remaining_time_s = 0.0f;
static int8_t g_enable_edf = (int8_t)1;

void EDF_setup(int edf_pin){
    pinMode(edf_pin, OUTPUT);
    EDF_motor.attach(edf_pin, 1148, 1832);
    EDF_motor.write((int)EDF_STANDSTILL_RAW_SPEED);
}

void EDF_activation_loop(float steering_angle_rad){
    int8_t edf_vehicle_min_speed_achived = (int8_t)0;
    g_enable_edf = (int8_t)1;
    if ((int)g_edf_raw_speed <= EDF_IDLE_RAW_SPEED) {
        g_enable_edf = (int8_t)0;
    }
    
    if (g_enable_car_engine != 0 && (floatCmp(g_car_speed_mps, EDF_MIN_VEHICLE_SPEED_MPS) >= 0) && (g_enable_edf != 0)) {
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
        edf_vehicle_min_speed_achived = (int8_t)1;
    }
    else {
        local_EDF_active_remaining_time_s = 0.0f;
        local_EDF_active = (int8_t)0;
    }
    
    if (local_EDF_active != (int8_t)0) {
        EDF_motor.write((int)g_edf_raw_speed);
    }
    else{
        if (edf_vehicle_min_speed_achived != (int8_t)0) {
            EDF_motor.write((int)EDF_IDLE_RAW_SPEED);
        }
        else{
            EDF_motor.write((int)EDF_STANDSTILL_RAW_SPEED);
        }
    }
}