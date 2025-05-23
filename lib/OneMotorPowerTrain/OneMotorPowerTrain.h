#ifndef __ONEMOTORPOWERTRAIN_H__
#define __ONEMOTORPOWERTRAIN_H__

#include "Arduino.h"
#include "PWMServo.h"
#include "esc_raw.h"


class OneMotorPowerTrain
{
private:
    /* data */
public:
    OneMotorPowerTrain(float wheelDiameter_meters, PWMServo motor_);
    ~OneMotorPowerTrain();
    void SetSpeedRequest(float speed_ms);
    void SetSpeedRequest_slow(float speed_ms);
    float MpsToRPM(float speedMps);
    void attach(int pin, int min, int max);
    void SetDiameter(float wheel_diameter);

    float GetSpeedRequest_raw();
    
    float GetSpeedRequest();

    float wheelDiameter_meters;
    float m_requested_speed_ms;
    float m_requested_raw_speed;
    PWMServo &motor;
};

extern OneMotorPowerTrain g_onemotorpowertrain;
void OneMotorPowerTrainSetup(float wheel_diameter_m, int motor_pin);


#endif