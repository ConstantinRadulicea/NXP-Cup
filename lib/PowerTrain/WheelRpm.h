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

#ifndef __WHEELRPM_H__
#define __WHEELRPM_H__

#include <Arduino.h>
#include <util/atomic.h>
#include "MedianFilter.h"
#include <geometry2D.h>


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
#define MillisToSec(val) ((val) / 1000)



void enableCpuCyclesCount();
unsigned long getCpuCycles();

typedef struct RpmSensorData {
    unsigned long TotalInterrupts;
    float TotalRotations;
    float Rpm;
    unsigned long LastSampleTimestamp_us;
    MedianFilter RpmAverage;
    void (*on_pulse)(volatile struct RpmSensorData *data);
    float TimePassedFromLastSample_us;
    int PulsePin;
    char LastState;
    float RpmFiltered; 
} RpmSensorData;

extern volatile RpmSensorData LeftWheelRpmData;

extern volatile RpmSensorData RightWheelRpmData;

 void ISR_RpmSensor(volatile RpmSensorData* data);
 RpmSensorData getRpmSensorData(volatile RpmSensorData* data);

 float getRpmPulsePeriod_us(float Rpm);

 float getRpm(volatile RpmSensorData* data);

 float getRpmFiltered(volatile RpmSensorData* data);

 float getRpm_adjusted(volatile RpmSensorData* data);

 float getRpmFiltered_adjusted(volatile RpmSensorData* data);

 float getTimePassedFromLastSample_us_adjusted(volatile RpmSensorData* data);

float getCurrentRpm_adjusted(volatile RpmSensorData* data);

 float getTotalRotations(volatile RpmSensorData* data);

 void setRpmSensorData(volatile RpmSensorData* dst, const RpmSensorData src);

/*===================================================================================================*/

 ENCODER_ISR_ATTR void ISR_RpmSensorLeftWheel();

 ENCODER_ISR_ATTR void ISR_RpmSensorRightWheel();

 RpmSensorData getLeftWheelRpmData();

 float getLeftWheelRpm();

 float getRightWheelRpm();

 float getLeftWheelTotalRotations();

 float getRightWheelTotalRotations();

 RpmSensorData getRightWheelRpmData();
 void setLeftWheelRpmData(const RpmSensorData val);
 void setRightWheelRpmData(const RpmSensorData val);
#endif
