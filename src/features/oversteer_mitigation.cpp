#include "features/oversteer_mitigation.h"
#include "PurePursuitGeometry.h"
#include "GlobalVariables.h"
#include "string.h"
#include "features/imu_data.h"
#include "PowerTrain.h"
#include <math.h>

static volatile float prev_yaw_rate = 0.0f;
static volatile float yaw_rate_delta = 0.0f;


enum direction {
    LEFT,
    FORWARD,
    RIGHT
};

enum situation_type{
    NONE_TYPE,
    OVERSTEER_TYPE,
    UNDERSTEER_TYPE
};

enum direction getSteeringDirection(float value) {
    if (floatCmp(value, 0.0f) < 0) return RIGHT;
    if (floatCmp(value, 0.0f) > 0) return LEFT;
    return FORWARD;
}

enum direction getYawDirection(float value) {
    if (floatCmp(value, 0.0f) < 0) return RIGHT;
    if (floatCmp(value, 0.0f) > 0) return LEFT;
    return FORWARD;
}


static volatile float new_wheel_speed_request = 0.0f;


float estimateMomentOfInertia(float mass, float wheelbase, float trackWidth) {
    return (1.0f / 12.0f) * mass * (wheelbase * wheelbase + trackWidth * trackWidth);
}


float computeWheelDeceleration(
    float speed, float yaw_calc, float yaw_meas,
    float wheelbase, float trackWidth, float mass)
{
    float yaw_error = yaw_meas - yaw_calc;
    yaw_error = fabsf(yaw_error);
    float Iz = estimateMomentOfInertia(mass, wheelbase, trackWidth);
    float dv = (Iz * yaw_error) / (mass * trackWidth);

    return dv;
}


/*
yawRateTolerance    -> positive turning left
                    -> negative turning right

*/
enum WheelToBrake detectAndMitigateOversteer(
    float speed,
    float steeringAngle,       // in radians
    float wheelbase,           // in meters
    float measuredYawRate,     // in rad/s
    float yawRateTolerance,     // in rad/s
    float time_s,                // time passed from last sample
    int8_t osm_active           // state of the current osm if active or not
) {

    if (wheelbase == 0.0f) return NONE;
    enum situation_type situation = NONE_TYPE;
    enum direction steering_direction = getSteeringDirection(steeringAngle);
    enum direction yaw_steering_direction = getYawDirection(measuredYawRate);

    yaw_rate_delta = ((measuredYawRate - prev_yaw_rate) / time_s);
    prev_yaw_rate = measuredYawRate;

    float expectedYawRate = calculateYawRate(speed, wheelbase, steeringAngle);

    if (steering_direction == RIGHT) {
        expectedYawRate = -expectedYawRate;
    }
    else if(steering_direction == LEFT){
        expectedYawRate = expectedYawRate;
    }

    //new_wheel_speed_request = computeWheelDeceleration(g_car_speed_mps, expectedYawRate, measuredYawRate, WHEEL_BASE_M, TRACK_WIDTH_M, 1.1f);
    //new_wheel_speed_request = speed - new_wheel_speed_request;
    //new_wheel_speed_request = MAX(new_wheel_speed_request, 0.0f);
    new_wheel_speed_request = 0.0f;
    float yaw_error = 0.0f;

    if (steering_direction == yaw_steering_direction)
    {
        if (steering_direction == RIGHT)    // negative steering angle negative yaw
        {
            yaw_error = expectedYawRate - measuredYawRate;  // positive -> oversteer, negative -> understeer
            if (floatCmp(yaw_error, yawRateTolerance) >= 0) {
                situation = OVERSTEER_TYPE;
            }
        }
        else if (steering_direction == LEFT)    // positive steering angle positive yaw
        {
            yaw_error = expectedYawRate - measuredYawRate;  // negative -> oversteer, positive -> understeer
            if (floatCmp(yaw_error, -yawRateTolerance) <= 0) {
                situation = OVERSTEER_TYPE;
            }
        }

        if (osm_active == 0 && situation == OVERSTEER_TYPE) {
            float temp_delta = fabs(yaw_rate_delta);
            if (floatCmp(temp_delta, g_oversteer_mitigation_yaw_delta_tolerance_rad_s) >= 0 && (signbit(yaw_rate_delta) == signbit(measuredYawRate))) {
                situation = OVERSTEER_TYPE;
            }
            else{
                situation = NONE_TYPE;
            }
        }
    }

    if (situation == OVERSTEER_TYPE)
    {
        if (steering_direction == LEFT) {
            return REAR_RIGHT;
        }
        if (steering_direction == RIGHT) {
            return REAR_LEFT;
        }
    }
    
    return NONE;
}





OSM_out_t oversteer_mitigation(float time_s){
    OSM_out_t result;

    memset(&result, 0, sizeof(result));
    if (g_enable_oversteer_mitigation == 0) {
        g_oversteer_mitigation_active = 0;
        return result;
    }
    
    enum WheelToBrake action;
    if (imu_data_is_valid() == 0)
    {
        g_oversteer_mitigation_active = 0;
        return result;/* code */
    }

    float temp_yaw_tolerance = MAX(g_oversteer_mitigation_yaw_tolerance_rad_s, 0.0f);
    float measured_yaw_rate_rad_s = imu_get_yaw_rate_rad_s();


    //if(g_oversteer_mitigation_active != 0){
    //    temp_yaw_tolerance = temp_yaw_tolerance * 0.5f;
    //}
    action = detectAndMitigateOversteer(g_car_speed_mps, g_steering_angle_rad, WHEEL_BASE_M, measured_yaw_rate_rad_s, temp_yaw_tolerance, time_s, g_oversteer_mitigation_active);
    if (action == NONE) {
        result.active = 0;
        g_oversteer_mitigation_active = 0;
    }
    else{
        result.active = 1;
        g_oversteer_mitigation_active = 1;
        result.wheel_speed_request_m_s = new_wheel_speed_request;
    }
    
    result.wheel_to_brake = action;
    return result;
}


void OSM_routine(float time_s){
    #if ENABLE_OVERSTEER_MITIGATION != 0
    OSM_out_t osm_result = oversteer_mitigation(time_s);
    if (osm_result.active != 0)
    {
        if (osm_result.wheel_to_brake == REAR_LEFT) {
            g_powertrain.SetLeftWheelSpeedRequest(osm_result.wheel_speed_request_m_s);
        }
        else if(osm_result.wheel_to_brake == REAR_RIGHT){
            g_powertrain.SetRightWheelSpeedRequest(osm_result.wheel_speed_request_m_s);
        }
    }
    #endif
}
