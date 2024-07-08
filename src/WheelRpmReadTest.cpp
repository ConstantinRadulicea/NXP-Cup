#include <Arduino.h>
#include <util/atomic.h>
#include "WheelRpm.h"
#include "Config.h"

#define RPM_SENSOR_LEFT_WHEEL_PIN 10
#define RPM_SENSOR_RIGHT_WHEEL_PIN 11

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
    temp_LeftWheelRpmData = getLeftWheelRpmData();
    temp_RightWheelRpmData = getRightWheelRpmData();

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
