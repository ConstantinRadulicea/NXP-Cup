#include <Arduino.h>
#include <util/atomic.h>
#include "WheelRpm.h"
#include "Config.h"
#include "ReadSerial.h"
#include <vector>

#define RPM_SENSOR_LEFT_WHEEL_PIN 3
#define RPM_SENSOR_RIGHT_WHEEL_PIN 4
#define RIGHT_WHEEL_MOTOR_PIN 23
#define LEFT_WHEEL_MOTOR_PIN 24

volatile PWMServo RightMotor;
volatile PWMServo LeftMotor;

void WriteToMotor(volatile PWMServo* motor_, float angle_arg){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        motor_->write((int)angle_arg);
    }   
}


void WriteToRightMotor(float angle_arg){
    WriteToMotor(&RightMotor, angle_arg);
}

void WriteToLeftMotor(float angle_arg){
    WriteToMotor(&LeftMotor, angle_arg);
}

void on_pulse_right_motor(volatile struct RpmSensorData *data){
    RpmSensorData temp_RightWheelRpmData;
    temp_RightWheelRpmData = getRightWheelRpmData();
    //Serial.println(temp_RightWheelRpmData.Rpm);
}
void on_pulse_left_motor(volatile struct RpmSensorData *data){

}

void empty_function(){

}

void setup() {
    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRightWheelRpmData();
    temp_WheelRpmData.on_pulse = on_pulse_right_motor;
    temp_WheelRpmData.PulsePin = RPM_SENSOR_RIGHT_WHEEL_PIN;
    setRightWheelRpmData(temp_WheelRpmData);

    temp_WheelRpmData = getLeftWheelRpmData();
    temp_WheelRpmData.on_pulse = on_pulse_left_motor;
    temp_WheelRpmData.PulsePin = RPM_SENSOR_LEFT_WHEEL_PIN;
    setLeftWheelRpmData(temp_WheelRpmData);

    pinMode(RIGHT_WHEEL_MOTOR_PIN, OUTPUT);
    pinMode(LEFT_WHEEL_MOTOR_PIN, OUTPUT);
    pinMode(RPM_SENSOR_LEFT_WHEEL_PIN, INPUT_PULLDOWN);
    pinMode(RPM_SENSOR_RIGHT_WHEEL_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_LEFT_WHEEL_PIN), ISR_RpmSensorLeftWheel, RISING);
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_RIGHT_WHEEL_PIN), ISR_RpmSensorRightWheel, RISING);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        RightMotor.attach(RIGHT_WHEEL_MOTOR_PIN, 1148, 1832);
        LeftMotor.attach(LEFT_WHEEL_MOTOR_PIN, 1148, 1832);
        RightMotor.write((int)90);
        LeftMotor.write((int)90);
    }
    
    Serial.begin(115200);
}


RpmSensorData temp_LeftWheelRpmData;
RpmSensorData temp_RightWheelRpmData;
std::vector<char> line;
char* pEnd;
int resultSuccess;
float leftMotorRawSpeed = 90.0, rightMotorRawSpeed = 90.0;
void loop() {
    if (readRecordFromSerial(Serial, "\r\n", line)) {
        pEnd = line.data();
        leftMotorRawSpeed = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        rightMotorRawSpeed = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        line.clear();
    }

    temp_LeftWheelRpmData = getLeftWheelRpmData();
    temp_RightWheelRpmData = getRightWheelRpmData();
/*
    WriteToLeftMotor(leftMotorRawSpeed);
    Serial.print("Left");
    Serial.print("\t");
    Serial.print("Raw Speed: ");
    Serial.print(leftMotorRawSpeed);
    Serial.print("\t");
    Serial.print("Rpm: ");
    Serial.print(temp_LeftWheelRpmData.Rpm);
    Serial.print("\t");
    Serial.print("TotalRotations: ");
    Serial.print(getLeftWheelTotalRotations());
    Serial.println();
*/
    WriteToRightMotor(rightMotorRawSpeed);
    /*
    Serial.print("Right");
    Serial.print("\t");
    Serial.print("Raw Speed: ");
    Serial.print(rightMotorRawSpeed);
    Serial.print("\t");
    Serial.print("Rpm: ");
    Serial.print(temp_RightWheelRpmData.Rpm);
    Serial.print("\t");
    Serial.print("TotalRotations: ");
    Serial.print(getRightWheelTotalRotations());
    Serial.println();
    */

    Serial.println(temp_RightWheelRpmData.Rpm);
    //Serial.println(digitalRead(RPM_SENSOR_RIGHT_WHEEL_PIN));
    
    delay(10);
}
