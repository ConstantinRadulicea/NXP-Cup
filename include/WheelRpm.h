#ifndef __WHEELRPM_H__
#define __WHEELRPM_H__

#include <Arduino.h>
#include <util/atomic.h>

#define RPM_SENSOR_TOTAL_HOLES 20

typedef struct RpmSensorData {
    float TotalRotations;
    float Rpm;
    unsigned long LastSampleTimestamp_us;
} RpmSensorData;

static volatile RpmSensorData LeftWheelRpmData = {
    .TotalRotations = 0.0,
    .Rpm = 0.0,
    .LastSampleTimestamp_us = micros()
};

static volatile RpmSensorData RightWheelRpmData = {
    .TotalRotations = 0.0,
    .Rpm = 0.0,
    .LastSampleTimestamp_us = micros()
};

static void ISR_RpmSensor(volatile RpmSensorData* data){
    unsigned long elapsed_time_us, time_now_us;

    time_now_us = micros();
    elapsed_time_us = data->LastSampleTimestamp_us;
    data->LastSampleTimestamp_us = time_now_us;
    elapsed_time_us = time_now_us - elapsed_time_us;

    // Update the total rotations
    data->TotalRotations += 1.0 / (float)RPM_SENSOR_TOTAL_HOLES;

    // Calculate RPM only if elapsed_time_us is non-zero to prevent division by zero
    if (elapsed_time_us > 0) {
        data->Rpm = 60.0 / ((float)elapsed_time_us / 1000000.0);
    }
}

static RpmSensorData getRpmSensorData(volatile RpmSensorData* data){
    RpmSensorData temp_data;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        temp_data.LastSampleTimestamp_us = data->LastSampleTimestamp_us;
        temp_data.Rpm = data->Rpm;
        temp_data.TotalRotations = data->TotalRotations;
    }
    return temp_data;
}
static void setRpmSensorData(volatile RpmSensorData* dst, const RpmSensorData src){
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        dst->LastSampleTimestamp_us = src.LastSampleTimestamp_us;
        dst->Rpm = src.Rpm;
        dst->TotalRotations = src.TotalRotations;
    }
}

/*===================================================================================================*/

static void ISR_RpmSensorLeftWheel() {
    ISR_RpmSensor(&LeftWheelRpmData);
}

static void ISR_RpmSensorRightWheel() {
    ISR_RpmSensor(&RightWheelRpmData);
}

static RpmSensorData getLeftWheelRpmData(){
    return getRpmSensorData(&LeftWheelRpmData);
}
static RpmSensorData getRightWheelRpmData(){
    return getRpmSensorData(&RightWheelRpmData);
}
static void setLeftWheelRpmData(const RpmSensorData val){
    setRpmSensorData(&LeftWheelRpmData, val);
}
static void setRightWheelRpmData(const RpmSensorData val){
    setRpmSensorData(&RightWheelRpmData, val);
}
#endif
