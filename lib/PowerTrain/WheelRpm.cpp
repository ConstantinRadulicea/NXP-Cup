/*
* Copyright 2024 Constantin Dumitru Petre RĂDULICEA
* 
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*   http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include "WheelRpm.h"

#define MEDIAN_FILTER_SIZE 5 // 5


void enableCpuCyclesCount(){
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

unsigned long getCpuCycles(){
    return ARM_DWT_CYCCNT;
}


volatile RpmSensorData LeftWheelRpmData = {
   .TotalInterrupts = 0,
    .TotalRotations = 0.0,
    .Rpm = 0.0,
    .LastSampleTimestamp_us = micros(),
    .RpmAverage = MedianFilter(MEDIAN_FILTER_SIZE),
    .on_pulse = NULL,
    .TimePassedFromLastSample_us = 0.0,
    .PulsePin = -1,
    .LastState = 0,
    .RpmFiltered = 0.0
};

 volatile RpmSensorData RightWheelRpmData = {
    .TotalInterrupts = 0,
    .TotalRotations = 0.0,
    .Rpm = 0.0,
    .LastSampleTimestamp_us = micros(),
    .RpmAverage = MedianFilter(MEDIAN_FILTER_SIZE),
    .on_pulse = NULL,
    .TimePassedFromLastSample_us = 0.0,
    .PulsePin = -1,
    .LastState = 0,
    .RpmFiltered = 0.0
};

 void ISR_RpmSensor(volatile RpmSensorData* data){
    unsigned long elapsed_time_us, time_now_us;
    uint8_t curValue;
    float local_rpm;
    time_now_us = micros();

    
    if(time_now_us < data->LastSampleTimestamp_us){
        data->LastSampleTimestamp_us = time_now_us;
        data->LastImpulseTimestamp_us = time_now_us;
        return;
    }
    elapsed_time_us = time_now_us - data->LastSampleTimestamp_us;
    if ((time_now_us - data->LastImpulseTimestamp_us) < (unsigned long)MillisToMicros(0.5f)) {
        data->LastImpulseTimestamp_us = time_now_us;
        return;
    }
    //if (elapsed_time_us < (unsigned long)MillisToMicros(0.5f)) {
    //    //data->LastSampleTimestamp_us = time_now_us;
    //    return;
    //}
    
    data->TotalInterrupts += 1;
    data->LastSampleTimestamp_us = time_now_us;
    data->LastImpulseTimestamp_us = time_now_us;

    // Update the total rotations
    data->TotalRotations += (1.0 / (float)RPM_SENSOR_PULSES_PER_REVOLUTION);

    data->TimePassedFromLastSample_us = (float)elapsed_time_us;
    local_rpm = (float)MillisToMicros(60*1000) / (float)((float)(elapsed_time_us) * (float)RPM_SENSOR_PULSES_PER_REVOLUTION);
    data->Rpm = local_rpm;
    data->RpmFiltered = data->RpmAverage.nextVolatile(local_rpm);
    
    if (data->on_pulse != NULL) {
        data->on_pulse(data);
    }
}

 RpmSensorData getRpmSensorData(volatile RpmSensorData* data){
    RpmSensorData temp_data;
        //ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
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
    //}
    return temp_data;
}

 float getRpmPulsePeriod_us(float Rpm){
    float period;
    period =(1.0/(Rpm/60.0))*1000000.0;
    return period;
}

 float getRpm(volatile RpmSensorData* data){
    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRpmSensorData(data);
    return temp_WheelRpmData.Rpm;
}

 float getRpmFiltered(volatile RpmSensorData* data){
    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRpmSensorData(data);
    return temp_WheelRpmData.RpmFiltered;
}

 float getRpm_adjusted(volatile RpmSensorData* data){
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

 float getRpmFiltered_adjusted(volatile RpmSensorData* data){
    unsigned long elapsed_time_us, time_now_us, micros_per_rpm;
    float result_rpm;
    RpmSensorData temp_WheelRpmData;

    time_now_us = micros();
    temp_WheelRpmData = getRpmSensorData(data);

    if (floatCmp(temp_WheelRpmData.RpmFiltered, 0.0) == 0) {
        return 0.0f;
    }
    

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

 float getTimePassedFromLastSample_us_adjusted(volatile RpmSensorData* data){
    unsigned long elapsed_time_us, time_now_us;
    RpmSensorData temp_WheelRpmData;

    time_now_us = micros();
    temp_WheelRpmData = getRpmSensorData(data);

    if(time_now_us < temp_WheelRpmData.LastSampleTimestamp_us){
        elapsed_time_us = (float)temp_WheelRpmData.LastSampleTimestamp_us - (float)time_now_us + temp_WheelRpmData.TimePassedFromLastSample_us;
        //elapsed_time_us = (float)temp_WheelRpmData.LastSampleTimestamp_us - (float)time_now_us;
    }
    else{
        elapsed_time_us = (float)time_now_us - (float)temp_WheelRpmData.LastSampleTimestamp_us + temp_WheelRpmData.TimePassedFromLastSample_us;
        //elapsed_time_us = (float)time_now_us - (float)temp_WheelRpmData.LastSampleTimestamp_us;

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



 float getTotalRotations(volatile RpmSensorData* data){
    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRpmSensorData(data);
    return temp_WheelRpmData.TotalRotations;
}

 void setRpmSensorData(volatile RpmSensorData* dst, const RpmSensorData src){
        //ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        dst->LastSampleTimestamp_us = src.LastSampleTimestamp_us;
        dst->Rpm = src.Rpm;
        dst->TotalRotations = src.TotalRotations;
        dst->TotalInterrupts = src.TotalInterrupts;
        dst->TimePassedFromLastSample_us = src.TimePassedFromLastSample_us;
        dst->on_pulse = src.on_pulse;
        dst->PulsePin = src.PulsePin;
        dst->RpmFiltered = src.RpmFiltered;
        dst->LastState = src.LastState;
    //}
}

/*===================================================================================================*/

 ENCODER_ISR_ATTR void ISR_RpmSensorLeftWheel() {
    ISR_RpmSensor(&LeftWheelRpmData);
}

 ENCODER_ISR_ATTR void ISR_RpmSensorRightWheel() {
    ISR_RpmSensor(&RightWheelRpmData);
    //asm("dsb");
}

 RpmSensorData getLeftWheelRpmData(){
    return getRpmSensorData(&LeftWheelRpmData);
}

 float getLeftWheelRpm(){
    return getRpm(&LeftWheelRpmData);
}

 float getRightWheelRpm(){
    return getRpm(&RightWheelRpmData);
}

 float getLeftWheelTotalRotations(){
    return getTotalRotations(&LeftWheelRpmData);
}

 float getRightWheelTotalRotations(){
    return getTotalRotations(&RightWheelRpmData);
}

 RpmSensorData getRightWheelRpmData(){
    return getRpmSensorData(&RightWheelRpmData);
}
 void setLeftWheelRpmData(const RpmSensorData val){
    setRpmSensorData(&LeftWheelRpmData, val);
}
 void setRightWheelRpmData(const RpmSensorData val){
    setRpmSensorData(&RightWheelRpmData, val);
}

