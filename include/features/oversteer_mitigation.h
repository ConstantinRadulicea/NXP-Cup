#ifndef __OVERSTEER_MITIGATION_H__
#define __OVERSTEER_MITIGATION_H__

#include "stdint.h"

enum WheelToBrake {
    NONE = 0,
    REAR_LEFT,
    REAR_RIGHT,
    BOTH
};


typedef struct OSM_out_s{
    int8_t enabled;
    int8_t active;
    enum WheelToBrake wheel_to_brake;
    float wheel_speed_request_m_s;
}OSM_out_t;

enum WheelToBrake detectAndMitigateOversteer(
    float speed,
    float steeringAngle,       // in radians
    float wheelbase,           // in meters
    float measuredYawRate,     // in rad/s
    float yawRateTolerance,     // in rad/s
    float time_s,                // time passed from last sample
    int8_t osm_active           // state of the current osm if active or not
);

OSM_out_t oversteer_mitigation(float time_s);

void OSM_routine(float time_s);

#endif