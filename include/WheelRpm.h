#ifndef __WHEELRPM_H__
#define __WHEELRPM_H__

#include <Arduino.h>
#include <util/atomic.h>
#include "MedianFilter.h"

// Use ICACHE_RAM_ATTR for ISRs to prevent ESP8266 resets
#if defined(ESP8266) || defined(ESP32)
#define ENCODER_ISR_ATTR ICACHE_RAM_ATTR
#else
#define ENCODER_ISR_ATTR
#endif

#define RPM_SENSOR_PULSES_PER_REVOLUTION (20)
#define MillisToMicros(val) ((val)*1000)
#define MicrosToMillis(val) ((val)/1000)
#define MicrosToSec(val) ((val)/1000000)
#define SecToMicros(val) ((val)*1000000)


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
    MedianFilter RpmAverage;
    void (*on_pulse)(volatile struct RpmSensorData *data);
    float TimePassedFromLastSample_us;
    int PulsePin;
    uint8_t LastState;
    float RpmFiltered; 
} RpmSensorData;

static volatile RpmSensorData LeftWheelRpmData = {
    .TotalInterrupts = 0,
    .TotalRotations = 0.0,
    .Rpm = 0.0,
    .LastSampleTimestamp_us = micros(),
    .RpmAverage = MedianFilter(6),
    .on_pulse = NULL,
    .TimePassedFromLastSample_us = 0.0,
    .PulsePin = -1,
    .LastState = 0,
    .RpmFiltered = 0.0
};

static volatile RpmSensorData RightWheelRpmData = {
    .TotalInterrupts = 0,
    .TotalRotations = 0.0,
    .Rpm = 0.0,
    .LastSampleTimestamp_us = micros(),
    .RpmAverage = MedianFilter(6),
    .on_pulse = NULL,
    .TimePassedFromLastSample_us = 0.0,
    .PulsePin = -1,
    .LastState = 0,
    .RpmFiltered = 0.0
};

static void ISR_RpmSensor(volatile RpmSensorData* data){
    unsigned long elapsed_time_us, time_now_us;
    uint8_t curValue;
    float local_rpm;
    time_now_us = micros();
    /*
    curValue = digitalRead(data->PulsePin);


    //Serial.print(data->LastState);
    //Serial.print("\t");
    //Serial.println(curValue);

    switch (data->LastState)
    {
    case 0:
        if (curValue == LOW){
            return;
        }
        else{
            data->LastState = 1;
        }        
        break;
    case 1:
        if (curValue == HIGH){
            return;
        }
        else{
            data->LastState = 0;
        } 
        break;
    
    default:
        return;
    }

*/
    
    if(time_now_us < data->LastSampleTimestamp_us){
        data->LastSampleTimestamp_us = time_now_us;
        return;
    }
    elapsed_time_us = time_now_us - data->LastSampleTimestamp_us;
    if (elapsed_time_us < MillisToMicros(1)) {
        return;
    }
    
    data->TotalInterrupts += 1;
    //Serial.println(data->TotalInterrupts);
    data->LastSampleTimestamp_us = time_now_us;

    // Update the total rotations
    data->TotalRotations += (1.0 / (float)RPM_SENSOR_PULSES_PER_REVOLUTION);

        data->TimePassedFromLastSample_us = elapsed_time_us;
        local_rpm = (float)MillisToMicros(60*1000) / (float)((elapsed_time_us) * RPM_SENSOR_PULSES_PER_REVOLUTION);
        //data->Rpm = data->RpmAverage.nextVolatile(local_rpm);
        data->Rpm = local_rpm;
        data->RpmFiltered = data->RpmAverage.nextVolatile(local_rpm);
    
    if (data->on_pulse != NULL) {
        //Serial.println((unsigned long)data->on_pulse);
        data->on_pulse(data);
    }
}

static RpmSensorData getRpmSensorData(volatile RpmSensorData* data){
    RpmSensorData temp_data;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        temp_data.LastSampleTimestamp_us = data->LastSampleTimestamp_us;
        temp_data.Rpm = data->Rpm;
        temp_data.TotalRotations = data->TotalRotations;
        temp_data.TotalInterrupts = data->TotalInterrupts;
        temp_data.RpmAverage = MedianFilter(0);
        temp_data.on_pulse = data->on_pulse;
        temp_data.TimePassedFromLastSample_us = data->TimePassedFromLastSample_us;
        temp_data.PulsePin = data->PulsePin;
        temp_data.LastState = data->LastState;
        temp_data.RpmFiltered = data->RpmFiltered;
    }
    return temp_data;
}

static float getRpmPulsePeriod_us(float Rpm){
    float period;
    period =(1.0/(Rpm/60.0))*1000000.0;
    return period;
}

static float getRpm(volatile RpmSensorData* data){
    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRpmSensorData(data);
    return temp_WheelRpmData.Rpm;
}

static float getRpmFiltered(volatile RpmSensorData* data){
    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRpmSensorData(data);
    return temp_WheelRpmData.RpmFiltered;
}

static float getRpm_adjusted(volatile RpmSensorData* data){
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

static float getRpmFiltered_adjusted(volatile RpmSensorData* data){
    unsigned long elapsed_time_us, time_now_us, micros_per_rpm;
    float result_rpm;
    RpmSensorData temp_WheelRpmData;

    time_now_us = micros();
    temp_WheelRpmData = getRpmSensorData(data);

    micros_per_rpm = (unsigned long)((1.0/(temp_WheelRpmData.RpmFiltered/60.0))*1000000.0);
    
    

    if(time_now_us < temp_WheelRpmData.LastSampleTimestamp_us){
        elapsed_time_us = temp_WheelRpmData.LastSampleTimestamp_us - time_now_us;
    }
    else{
        elapsed_time_us = time_now_us - temp_WheelRpmData.LastSampleTimestamp_us;
    }

    if (elapsed_time_us > 0) {
        micros_per_rpm = micros_per_rpm + elapsed_time_us;
        result_rpm = (float)MillisToMicros(60*1000) / (float)((micros_per_rpm));
    }
    else{
        result_rpm = temp_WheelRpmData.RpmFiltered;
    }
    return result_rpm;
}

static float getTimePassedFromLastSample_us_adjusted(volatile RpmSensorData* data){
    unsigned long elapsed_time_us, time_now_us;
    RpmSensorData temp_WheelRpmData;

    time_now_us = micros();
    temp_WheelRpmData = getRpmSensorData(data);

    if(time_now_us < temp_WheelRpmData.LastSampleTimestamp_us){
        elapsed_time_us = (float)temp_WheelRpmData.LastSampleTimestamp_us - (float)time_now_us + temp_WheelRpmData.TimePassedFromLastSample_us;
    }
    else{
        elapsed_time_us = (float)time_now_us - (float)temp_WheelRpmData.LastSampleTimestamp_us + temp_WheelRpmData.TimePassedFromLastSample_us;
    }
    return elapsed_time_us;
}


float getCurrentRpm_adjusted(volatile RpmSensorData* data){
    float rpm, timePassed, timePassedAdjusted;
    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRpmSensorData(data);
    timePassedAdjusted = getTimePassedFromLastSample_us_adjusted(data);

    rpm = temp_WheelRpmData.RpmFiltered;
    timePassed = temp_WheelRpmData.TimePassedFromLastSample_us;

    if (/*(timePassedAdjusted > (5.0*timePassed)) || */timePassedAdjusted > MillisToMicros(500))
    {
        timePassed = timePassedAdjusted;
        rpm = 0.0;
    }
    else if (timePassedAdjusted > (5.0*timePassed)) {
        timePassed = timePassedAdjusted;
        rpm = getRpmFiltered_adjusted(data);
    }
    return rpm;
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
        dst->TimePassedFromLastSample_us = src.TimePassedFromLastSample_us;
        dst->on_pulse = src.on_pulse;
        dst->PulsePin = src.PulsePin;
        dst->RpmFiltered = src.RpmFiltered;
        dst->LastState = src.LastState;
    }
}

/*===================================================================================================*/

static ENCODER_ISR_ATTR void ISR_RpmSensorLeftWheel() {
    ISR_RpmSensor(&LeftWheelRpmData);
}

static ENCODER_ISR_ATTR void ISR_RpmSensorRightWheel() {
    ISR_RpmSensor(&RightWheelRpmData);
    //asm("dsb");
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
