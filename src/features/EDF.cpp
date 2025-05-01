

#include "features/EDF.h"
#include "Arduino.h"
#include <PWMServo.h>
#include "GlobalVariables.h"
#include "features/automatic_emergency_braking.h"

#define EDF_MIN_ACTIVE_TIME_S 1.0f
#define EDF_STEERING_ANGLE_ACTIVATION_RAD radians(0.0f)
#define EDF_MIN_VEHICLE_SPEED_MPS 0.5f
#define EDF_IDLE_RAW_SPEED g_edf_raw_speed
#define EDF_STANDSTILL_RAW_SPEED 90

#define EDF_STOP_MIN_OBSTACLE_DISTANCE_M 0.2f
#define EDF_ACTIVATION_COMPLETE_COUNTDOWN_S 5.0f

volatile PWMServo EDF_motor;
static int8_t local_EDF_active = (int8_t)0;
static float local_EDF_active_remaining_time_s = 0.0f;
static int8_t g_enable_edf = (int8_t)1;

static float local_EDF_activation_complete_remaining_time_s = 0.0f;
static int8_t locl_EDF_activation_started = (int8_t)0;
static int8_t locl_EDF_activation_completed = (int8_t)0;

void EDF_setup(int edf_pin){
    pinMode(edf_pin, OUTPUT);
    EDF_motor.attach(edf_pin, 1148, 1832);
    EDF_motor.write((int)EDF_STANDSTILL_RAW_SPEED);
}

void EDF_activation_loop(float steering_angle_rad){
    int8_t edf_vehicle_min_speed_achived = (int8_t)0;
    float temp_obstacle_distance = 0.0f;
    float p_EDF_raw_speed = EDF_STANDSTILL_RAW_SPEED;
    

    if (g_enable_car_engine == (int8_t)0 && locl_EDF_activation_completed != (int8_t)0) {
        g_enable_edf = (int8_t)1;
        locl_EDF_activation_started = 0;
        locl_EDF_activation_completed = 0;
    }


    if (g_enable_car_engine != (int8_t)0 && locl_EDF_activation_started == (int8_t)0) {

        local_EDF_activation_complete_remaining_time_s = 0.0f;
        if ((int)g_edf_raw_speed > (int)EDF_STANDSTILL_RAW_SPEED) {
            local_EDF_activation_complete_remaining_time_s = EDF_ACTIVATION_COMPLETE_COUNTDOWN_S;
        }

        locl_EDF_activation_started = (int8_t)1;        
    }

    locl_EDF_activation_completed = 0;
    if (locl_EDF_activation_started != 0) {
        local_EDF_activation_complete_remaining_time_s -= MillisToSec(g_loop_time_ms);
        local_EDF_activation_complete_remaining_time_s = MAX(local_EDF_activation_complete_remaining_time_s, -1.0f);
        if (local_EDF_activation_complete_remaining_time_s <= 0.0f) {
            locl_EDF_activation_completed = 1;
            g_enable_car_engine = (int8_t)1;
        }
    }

    if (g_enable_car_engine != (int8_t)0 && locl_EDF_activation_started != (int8_t)0 && locl_EDF_activation_completed == (int8_t)0) {
        g_enable_car_engine = (int8_t)0;
    }

    if (locl_EDF_activation_completed != 0) {
        g_enable_car_engine = (int8_t)1;
    }
    
    


    if (g_enable_car_engine != 0)
    {
        if (g_valid_track_lines_flag == (int8_t)0 || g_valid_vectors_detected_flag == (int8_t)0) {
            g_enable_edf = (int8_t)0;
        }
        temp_obstacle_distance = getFrontObstacleDistanceAnalog_m();
        if (floatCmp(temp_obstacle_distance, EDF_STOP_MIN_OBSTACLE_DISTANCE_M) <= 0) {
            g_enable_edf = (int8_t)0;
        }

        if (g_finish_line_detected_slowdown != 0) {
            g_enable_edf = (int8_t)0;
        }
        
    }
    

    //if ((int)g_edf_raw_speed <= EDF_IDLE_RAW_SPEED) {
    //    g_enable_edf = (int8_t)0;
    //}
    
    if (locl_EDF_activation_started != 0 && (floatCmp(g_car_speed_mps, EDF_MIN_VEHICLE_SPEED_MPS) >= 0) && (g_enable_edf != 0)) {
        if (local_EDF_active != (int8_t)0) {
            local_EDF_active_remaining_time_s -= MillisToSec(g_loop_time_ms);
            local_EDF_active_remaining_time_s = MAX(local_EDF_active_remaining_time_s, -1.0f);
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
        p_EDF_raw_speed = MAX(g_edf_raw_speed, (float)EDF_STANDSTILL_RAW_SPEED);
        EDF_motor.write((int)p_EDF_raw_speed);
    }
    else{
        if (edf_vehicle_min_speed_achived != (int8_t)0) {
            p_EDF_raw_speed = MAX(EDF_IDLE_RAW_SPEED, (float)EDF_STANDSTILL_RAW_SPEED);
            EDF_motor.write((int)p_EDF_raw_speed);
        }
        else{
            EDF_motor.write((int)EDF_STANDSTILL_RAW_SPEED);
        }
    }
}