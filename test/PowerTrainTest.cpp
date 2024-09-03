#include <Arduino.h>
#include <vector>
#include <util/atomic.h>
#include "WheelRpm.h"
#include "Config.h"
#include "ReadSerial.h"
#include "PowerTrain.h"
#include <vector>

#define POWERTRAIN_PID_FREQUENCY_HZ 100
//#define RPM_SENSOR_LEFT_WHEEL_PIN 2
//#define RPM_SENSOR_RIGHT_WHEEL_PIN 4
//#define RIGHT_WHEEL_MOTOR_PIN 23
//#define LEFT_WHEEL_MOTOR_PIN 22
#define WHEEL_DIAMETER_M 0.064	//wheel diameter im meters
#define DISTANCE_BETWEEN_WHEELS_M 0.145	//distance between wheels

float kp = 0.0;
float ki = 0.1;
float kd = 0.0;
float ki_sum = 0.5;


void setup() {
    PowerTrainSetup(WHEEL_DIAMETER_M, DISTANCE_BETWEEN_WHEELS_M, POWERTRAIN_PID_FREQUENCY_HZ, LEFT_WHEEL_MOTOR_PIN, RIGHT_WHEEL_MOTOR_PIN, RPM_SENSOR_LEFT_WHEEL_PIN, RPM_SENSOR_RIGHT_WHEEL_PIN);
    Serial.begin(SERIAL_PORT_BAUD_RATE);
    //while (!Serial) { delay(50); }
}


RpmSensorData temp_LeftWheelRpmData;
RpmSensorData temp_RightWheelRpmData;

RpmSensorData temp_WheelRpmData;
std::vector<char> line;
char* pEnd;
int resultSuccess;
float leftMotorRawSpeed = 0.0, direction = 0.0, carTurnRadius = 0.0;
void loop() {
    if (readRecordFromSerial(Serial, "\r\n", line)) {
        // 1;0;0;0;0.1;0;0.5\r\n
        pEnd = line.data();
        //Serial.println(line.data());
        leftMotorRawSpeed = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        direction = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        carTurnRadius = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        kp = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        ki = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        kd = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        ki_sum = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            powerTrain.SetLeftWheelPID(kp, ki, kd, ki_sum);
            powerTrain.SetRightWheelPID(kp, ki, kd, ki_sum);
            //powerTrain.SetLeftWheelSpeedRequest_volatile(leftMotorRawSpeed);
            //powerTrain.SetRightWheelSpeedRequest_volatile(rightMotorRawSpeed);
            powerTrain.SetSpeedRequest_volatile(leftMotorRawSpeed, carTurnRadius, direction);
        }
        line.clear();
    }


    temp_WheelRpmData = getLeftWheelRpmData();
    Serial.print(powerTrain.GetLeftWheelSpeed());
    Serial.print(';');
    Serial.print(powerTrain.GetLeftWheelSpeedRequest_raw());
    Serial.print(';');
    Serial.print(temp_WheelRpmData.Rpm);
    Serial.print(';');
    
    temp_WheelRpmData = getRightWheelRpmData();
    Serial.print(powerTrain.GetRightWheelSpeed());
    Serial.print(';');
    Serial.print(powerTrain.GetRightWheelSpeedRequest_raw());
    Serial.print(';');
    Serial.print(temp_WheelRpmData.Rpm);

    Serial.println();


    delay(50);
}
