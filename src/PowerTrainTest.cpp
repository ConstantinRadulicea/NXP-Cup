#include <Arduino.h>
#include <vector>
#include <util/atomic.h>
#include "WheelRpm.h"
#include "Config.h"
#include "ReadSerial.h"
#include "PowerTrain.h"
#include <vector>

float kp = 0.0;
float ki = 3.0;
float kd = 0.0;
float ki_sum = 0.3;


void setup() {
    PowerTrainSetup();
    Serial.begin(256000);
    //while (!Serial) { delay(50); }
}


RpmSensorData temp_LeftWheelRpmData;
RpmSensorData temp_RightWheelRpmData;
std::vector<char> line;
char* pEnd;
int resultSuccess;
float leftMotorRawSpeed = 0.0, rightMotorRawSpeed = 0.0;
void loop() {
    if (readRecordFromSerial(Serial, "\r\n", line)) {
        // 1;2;3;4;5
        pEnd = line.data();
        Serial.println(line.data());
        rightMotorRawSpeed = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        kp = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        ki = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        kd = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        ki_sum = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);

        //Serial.println(rightMotorRawSpeed);
        //Serial.println(kp);
        //Serial.println(ki);
        //Serial.println(kd);
        //Serial.println(ki_sum);

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            powerTrain.SetRightWheelPID(kp, ki, kd, ki_sum);
            powerTrain.SetRightWheelSpeedRequest_volatile(rightMotorRawSpeed);
        }
        line.clear();
    }
    delay(100);
}
