#ifndef __WHEELRPM_H__
#define __WHEELRPM_H__

#include <Arduino.h>
#include <util/atomic.h>
#include "MovingAverage.h"

#define RPM_SENSOR_PULSES_PER_REVOLUTION (20*2)
#define MillisToMicros(val) (val*1000)

void enableCpuCyclesCount(){
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

unsigned long getCpuCycles(){
    return ARM_DWT_CYCCNT;
}

typedef struct RpmSensorData {
    unsigned long TotalInterrupts;
    float TotalRotations;
    float Rpm;
    unsigned long LastSampleTimestamp_us;
    MovingAverage RpmAverage;
} RpmSensorData;

static volatile RpmSensorData LeftWheelRpmData = {
    .TotalInterrupts = 0,
    .TotalRotations = 0.0,
    .Rpm = 0.0,
    .LastSampleTimestamp_us = micros(),
    .RpmAverage = MovingAverage(5)
};

static volatile RpmSensorData RightWheelRpmData = {
    .TotalInterrupts = 0,
    .TotalRotations = 0.0,
    .Rpm = 0.0,
    .LastSampleTimestamp_us = micros(),
    .RpmAverage = MovingAverage(20)
};

static void ISR_RpmSensor(volatile RpmSensorData* data){
    unsigned long elapsed_time_us, time_now_us;
    float local_rpm;
    
    
    time_now_us = micros();
    if(time_now_us < data->LastSampleTimestamp_us){
        data->LastSampleTimestamp_us = time_now_us;
        return;
    }
    elapsed_time_us = time_now_us - data->LastSampleTimestamp_us;
    if (elapsed_time_us < MillisToMicros(0.5)) {
        return;
    }
    
    data->TotalInterrupts += 1,
    data->LastSampleTimestamp_us = time_now_us;

    // Update the total rotations
    data->TotalRotations += (1.0 / (float)RPM_SENSOR_PULSES_PER_REVOLUTION);

    // Calculate RPM only if elapsed_time_us is non-zero to prevent division by zero
    if (elapsed_time_us > 0) {
        local_rpm = (float)MillisToMicros(60*1000) / (float)((elapsed_time_us) * RPM_SENSOR_PULSES_PER_REVOLUTION);
        data->Rpm = data->RpmAverage.nextVolatile(local_rpm);
        //data->Rpm = local_rpm;
    }
}

static RpmSensorData getRpmSensorData(volatile RpmSensorData* data){
    RpmSensorData temp_data;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        temp_data.LastSampleTimestamp_us = data->LastSampleTimestamp_us;
        temp_data.Rpm = data->Rpm;
        temp_data.TotalRotations = data->TotalRotations;
        temp_data.TotalInterrupts = data->TotalInterrupts;
        temp_data.RpmAverage = MovingAverage(0);
    }
    return temp_data;
}

static float getRpm(volatile RpmSensorData* data){
    unsigned long elapsed_time_us, time_now_us;
    float result_rpm;

    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRpmSensorData(data);
    
    time_now_us = micros();

    if(time_now_us < temp_WheelRpmData.LastSampleTimestamp_us){
        elapsed_time_us = temp_WheelRpmData.LastSampleTimestamp_us - time_now_us;
    }
    else{
        elapsed_time_us = time_now_us - temp_WheelRpmData.LastSampleTimestamp_us;
    }

    if (elapsed_time_us > 0) {
        result_rpm = (float)MillisToMicros(60*1000) / (float)((elapsed_time_us) * RPM_SENSOR_PULSES_PER_REVOLUTION);
    }
    else{
        result_rpm = temp_WheelRpmData.Rpm;
    }
    return result_rpm;
}

static float getTotalRotations(volatile RpmSensorData* data){
    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRpmSensorData(data);
    return temp_WheelRpmData.TotalRotations;
}

static void setRpmSensorData(volatile RpmSensorData* dst, const RpmSensorData src){
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        dst->LastSampleTimestamp_us = src.LastSampleTimestamp_us;
        dst->Rpm = src.Rpm;
        dst->TotalRotations = src.TotalRotations;
        dst->TotalInterrupts = src.TotalInterrupts;
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

static float getLeftWheelRpm(){
    return getRpm(&LeftWheelRpmData);
}

static float getRightWheelRpm(){
    return getRpm(&RightWheelRpmData);
}

static float getLeftWheelTotalRotations(){
    return getTotalRotations(&LeftWheelRpmData);
}

static float getRightWheelTotalRotations(){
    return getTotalRotations(&RightWheelRpmData);
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
