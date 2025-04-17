#include "features/oversteer_mitigation.h"
#include "PurePursuitGeometry.h"
#include "GlobalVariables.h"
#include "string.h"
#include "features/imu_data.h"
#include "PowerTrain.h"


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


/*
yawRateTolerance    -> positive turning left
                    -> negative turning right

*/
enum WheelToBrake detectAndMitigateOversteer(
    float speed,
    float steeringAngle,       // in radians
    float wheelbase,           // in meters
    float measuredYawRate,     // in rad/s
    float yawRateTolerance     // in rad/s
) {
    if (wheelbase == 0.0f) return NONE;
    enum situation_type situation = NONE_TYPE;
    enum direction steering_direction = getSteeringDirection(steeringAngle);
    enum direction yaw_steering_direction = getYawDirection(measuredYawRate);

    float expectedYawRate = calculateYawRate(speed, wheelbase, steeringAngle);

    if (steering_direction == RIGHT) {
        expectedYawRate = -expectedYawRate;
    }
    else if(steering_direction == LEFT){
        expectedYawRate = expectedYawRate;
    }

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


OSM_out_t oversteer_mitigation(){
    OSM_out_t result;

    memset(&result, 0, sizeof(result));
    if (g_enable_oversteer_mitigation == 0) {
        g_oversteer_mitigation_active = 0;
        return result;
    }
    
    enum WheelToBrake action;
    if (imu_data_is_valid() == 0)
    {
        return result;/* code */
    }
    float measured_yaw_rate_rad_s = imu_get_yaw_rate_rad_s();
    action = detectAndMitigateOversteer(g_car_speed_mps, g_steering_angle_rad, WHEEL_BASE_M, measured_yaw_rate_rad_s, g_oversteer_mitigation_rad_s);
    if (action == NONE) {
        result.active = 0;
        g_oversteer_mitigation_active = 0;
    }
    else{
        result.active = 1;
        g_oversteer_mitigation_active = 1;
    }
    
    result.wheel_to_brake = action;
    return result;
}


void OSM_routine(){
    #if ENABLE_OVERSTEER_MITIGATION != 0
    OSM_out_t osm_result = oversteer_mitigation();
    if (osm_result.active != 0)
    {
        if (osm_result.wheel_to_brake == REAR_LEFT) {
            g_powertrain.SetLeftWheelSpeedRequest(0.0f);
        }
        else if(osm_result.wheel_to_brake == REAR_RIGHT){
            g_powertrain.SetRightWheelSpeedRequest(0.0f);
        }
    }
    #endif
}
