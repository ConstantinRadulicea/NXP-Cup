#include "features/oversteer_mitigation.h"
#include "PurePursuitGeometry.h"
#include "GlobalVariables.h"
#include "string.h"
#include "features/imu_data.h"
#include "PowerTrain.h"


enum steer_direction {
    LEFT,
    FORWARD,
    RIGHT
};


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

    enum steer_direction steering_direction = FORWARD;
    enum steer_direction yaw_steering_direction = FORWARD;

    float expectedYawRate = calculateYawRate(speed, wheelbase, steeringAngle);

    if(floatCmp(steeringAngle, 0.0f) < 0){  // turning right
        steering_direction = RIGHT;
    }
    else if (floatCmp(steeringAngle, 0.0f) == 0) {
        steering_direction = FORWARD;
    }
    else{
        steering_direction = LEFT;
    }
    if (steering_direction == RIGHT) {
        expectedYawRate = -expectedYawRate;
    }

    if(floatCmp(measuredYawRate, 0.0f) < 0){  // turning right
        yaw_steering_direction = RIGHT;
    }
    else if (floatCmp(measuredYawRate, 0.0f) == 0) {
        yaw_steering_direction = FORWARD;
    }
    else{
        yaw_steering_direction = LEFT;
    }
    if (yaw_steering_direction == RIGHT) {
    }
    

    float yawError = measuredYawRate - expectedYawRate;
    
    yawError = fabs(yawError);

    if (yawError > yawRateTolerance) {
        if (steering_direction == RIGHT)
        {
            if (yaw_steering_direction == RIGHT) {
                return REAR_LEFT;
            }
        }
        else if (steering_direction == LEFT)
        {
            if (yaw_steering_direction == LEFT) {
                return REAR_RIGHT;
            }
        }
    }

    return NONE;
}


OSM_out_t oversteer_mitigation(){
    OSM_out_t result;

    memset(&result, 0, sizeof(result));
    if (g_enable_oversteer_mitigation == 0) {
        return result;
    }
    
    enum WheelToBrake action;
    if (imu_data_is_valid() == 0)
    {
        return result;/* code */
    }
    float measured_yaw_rate_rad_s = imu_get_yaw_rate_rad_s();
    action = detectAndMitigateOversteer(g_car_speed_mps, g_steering_angle_rad, WHEEL_BASE_M, measured_yaw_rate_rad_s, radians(50.0f));
    if (action == NONE) {
        result.active = 0;
    }
    else{
        result.active = 1;
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
