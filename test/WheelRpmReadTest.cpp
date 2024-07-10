#include <Arduino.h>
#include <util/atomic.h>
#include "WheelRpm.h"
#include "Config.h"


#define RPM_SENSOR_LEFT_WHEEL_PIN 2
#define RPM_SENSOR_RIGHT_WHEEL_PIN 3

void setup() {
    Serial.begin(115200);
    pinMode(RPM_SENSOR_LEFT_WHEEL_PIN, INPUT);
    pinMode(RPM_SENSOR_RIGHT_WHEEL_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_LEFT_WHEEL_PIN), ISR_RpmSensorLeftWheel, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_RIGHT_WHEEL_PIN), ISR_RpmSensorRightWheel, CHANGE);

}


RpmSensorData temp_LeftWheelRpmData;
RpmSensorData temp_RightWheelRpmData;
void loop() {
    // Access the volatile struct atomically
    //temp_LeftWheelRpmData = getLeftWheelRpmData();
    temp_RightWheelRpmData = getRightWheelRpmData();

/*
    Serial.print("Left");
    Serial.print("\t");
    Serial.print("N: ");
    Serial.print(getLeftWheelRpm());
    Serial.print("\t");
    Serial.print("Rpm: ");
    Serial.print(temp_LeftWheelRpmData.Rpm);
    Serial.print("\t");
    Serial.print("TotalRotations: ");
    Serial.print(getLeftWheelTotalRotations());
    Serial.print("\t");
    Serial.print("LastSampleTimestamp_us: ");
    Serial.print(temp_LeftWheelRpmData.LastSampleTimestamp_us);
    Serial.println();
*/
    Serial.print("Right");
    Serial.print("\t");
    Serial.print("N: ");
    Serial.print(temp_RightWheelRpmData.TotalInterrupts);
    Serial.print("\t");
    Serial.print("Rpm: ");
    Serial.print(getRightWheelRpm());
    Serial.print("\t");
    Serial.print("TotalRotations: ");
    Serial.print(getRightWheelTotalRotations());
    Serial.print("\t");
    Serial.print("LastSampleTimestamp_us: ");
    Serial.print(temp_RightWheelRpmData.LastSampleTimestamp_us);
    Serial.println();
    delay(500);
}
