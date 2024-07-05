#include <Arduino.h>
#include <util/atomic.h>
#include "Config.h"

#define RPM_SENSOR_TOTAL_HOLES 20
#define RPM_SENSOR_LEFT_WHEEL_PIN 10
#define RPM_SENSOR_RIGHT_WHEEL_PIN 11

typedef struct RpmSensorData {
    float TotalRotations;
    float Rpm;
    unsigned long LastSampleTimestamp_us;
} RpmSensorData;

volatile RpmSensorData LeftWheelRpmData = {
    .TotalRotations = 0.0,
    .Rpm = 0.0,
    .LastSampleTimestamp_us = micros()
};

volatile RpmSensorData RightWheelRpmData = {
    .TotalRotations = 0.0,
    .Rpm = 0.0,
    .LastSampleTimestamp_us = micros()
};

void ISR_RpmSensorLeftWheel() {
    unsigned long elapsed_time_us, time_now_us;

    time_now_us = micros();
    elapsed_time_us = LeftWheelRpmData.LastSampleTimestamp_us;
    LeftWheelRpmData.LastSampleTimestamp_us = time_now_us;
    elapsed_time_us = time_now_us - elapsed_time_us;

    // Update the total rotations
    LeftWheelRpmData.TotalRotations += 1.0 / (float)RPM_SENSOR_TOTAL_HOLES;

    // Calculate RPM only if elapsed_time_us is non-zero to prevent division by zero
    if (elapsed_time_us > 0) {
        LeftWheelRpmData.Rpm = 60.0 / ((float)elapsed_time_us / 1000000.0);
    }
}

void ISR_RpmSensorRightWheel() {
    unsigned long elapsed_time_us, time_now_us;

    time_now_us = micros();
    elapsed_time_us = RightWheelRpmData.LastSampleTimestamp_us;
    RightWheelRpmData.LastSampleTimestamp_us = time_now_us;
    elapsed_time_us = time_now_us - elapsed_time_us;

    // Update the total rotations
    RightWheelRpmData.TotalRotations += 1.0 / (float)RPM_SENSOR_TOTAL_HOLES;

    // Calculate RPM only if elapsed_time_us is non-zero to prevent division by zero
    if (elapsed_time_us > 0) {
        RightWheelRpmData.Rpm = 60.0 / ((float)elapsed_time_us / 1000000.0);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(RPM_SENSOR_LEFT_WHEEL_PIN, INPUT);
    pinMode(RPM_SENSOR_RIGHT_WHEEL_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_LEFT_WHEEL_PIN), ISR_RpmSensorLeftWheel, RISING);
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_RIGHT_WHEEL_PIN), ISR_RpmSensorRightWheel, RISING);
}


RpmSensorData temp_LeftWheelRpmData;
RpmSensorData temp_RightWheelRpmData;
void loop() {
    // Access the volatile struct atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        temp_LeftWheelRpmData.LastSampleTimestamp_us = LeftWheelRpmData.LastSampleTimestamp_us;
        temp_LeftWheelRpmData.Rpm = LeftWheelRpmData.Rpm;
        temp_LeftWheelRpmData.TotalRotations = LeftWheelRpmData.TotalRotations;

        temp_RightWheelRpmData.LastSampleTimestamp_us = RightWheelRpmData.LastSampleTimestamp_us;
        temp_RightWheelRpmData.Rpm = RightWheelRpmData.Rpm;
        temp_RightWheelRpmData.TotalRotations = RightWheelRpmData.TotalRotations;
    }

    Serial.print("Left");
    Serial.print("\t");
    Serial.print("Rpm: ");
    Serial.print(temp_LeftWheelRpmData.Rpm);
    Serial.print("\t");
    Serial.print("TotalRotations: ");
    Serial.print(temp_LeftWheelRpmData.TotalRotations);
    Serial.print("\t");
    Serial.print("LastSampleTimestamp_us: ");
    Serial.print(temp_LeftWheelRpmData.LastSampleTimestamp_us);
    Serial.println();

    Serial.print("Right");
    Serial.print("\t");
    Serial.print("Rpm: ");
    Serial.print(temp_RightWheelRpmData.Rpm);
    Serial.print("\t");
    Serial.print("TotalRotations: ");
    Serial.print(temp_RightWheelRpmData.TotalRotations);
    Serial.print("\t");
    Serial.print("LastSampleTimestamp_us: ");
    Serial.print(temp_RightWheelRpmData.LastSampleTimestamp_us);
    Serial.println();
}
