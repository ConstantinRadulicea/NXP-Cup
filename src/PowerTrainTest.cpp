#include <Arduino.h>
#include <util/atomic.h>
#include "WheelRpm.h"
#include "Config.h"
#include "ReadSerial.h"
#include "PowerTrain.h"
#include <vector>

#define POWERTRAIN_PID_FREQUENCY_HZ 500
#define HzToSec(hz) (1.0/hz)

IntervalTimer myTimer;

#define RPM_SENSOR_LEFT_WHEEL_PIN 2
#define RPM_SENSOR_RIGHT_WHEEL_PIN 3
#define RIGHT_WHEEL_MOTOR_PIN 23
#define LEFT_WHEEL_MOTOR_PIN 24

float wheelDiameter = 0.064; // exemplu: diametru roata in metri

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


void WriteToRightMotorThrottle(float throttle){
    float rawValue;
    rawValue = 90.0 + (throttle * 90.0);
    rawValue = MAX(rawValue, 90.0);
    rawValue = MIN(rawValue, 150.0);
    WriteToRightMotor(rawValue);
}

void WriteToLeftMotorThrottle(float throttle){
    float rawValue;
    rawValue = 90.0 + (throttle * 90.0);
    rawValue = MAX(rawValue, 90.0);
    rawValue = MIN(rawValue, 150.0);
    WriteToLeftMotor(rawValue);
}

volatile PowerTrain powerTrain(0.0, 0.0, 0.0, wheelDiameter, WriteToLeftMotorThrottle, WriteToRightMotorThrottle);


void on_pulse_right_motor(volatile struct RpmSensorData *data){
}

void on_pulse_left_motor(volatile struct RpmSensorData *data){
}


void power_train_sampling(){
    float rpm, timePassed;
    rpm = getRpm_adjusted(&RightWheelRpmData);
    timePassed = getTimePassedFromLastSample_us_adjusted(&RightWheelRpmData);
    if (timePassed > MillisToMicros(1000)) {
        rpm = 0.0;
    }
    powerTrain.SetRightWheelMeasuredRPM_volatile(rpm, timePassed);

    Serial.print("Rightgggg");
    Serial.print("\t");
    Serial.print("Raw Speed: ");
    Serial.print(powerTrain.GetRightWheelSpeedRequest_raw());
    Serial.print("\t");
    Serial.print("m/s: ");
    Serial.print(powerTrain.GetRightWheelSpeed());
    Serial.print("\t");
    Serial.print("Rpm: ");
    Serial.print(powerTrain.rightWheel.GetRpm());
    Serial.print("\t");
    Serial.print("TotalRotations: ");
    Serial.print(getRightWheelTotalRotations());
    Serial.print("\t");
    Serial.print("TimePassed: ");
    Serial.print(timePassed);
    Serial.println();

    rpm = getRpm_adjusted(&LeftWheelRpmData);
    timePassed = getTimePassedFromLastSample_us_adjusted(&LeftWheelRpmData);
    if (timePassed > MillisToMicros(1000)) {
        rpm = 0.0;
    }
    
    powerTrain.SetLeftWheelMeasuredRPM_volatile(rpm, timePassed);
}
void empty_function(){

}

void setup() {
    powerTrain.SetRightWheelPID(0.1, 1.8, 0.0, 0.5);
    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRightWheelRpmData();
    temp_WheelRpmData.on_pulse = on_pulse_right_motor;
    setRightWheelRpmData(temp_WheelRpmData);

    temp_WheelRpmData = getLeftWheelRpmData();
    temp_WheelRpmData.on_pulse = on_pulse_left_motor;
    setLeftWheelRpmData(temp_WheelRpmData);

    pinMode(RIGHT_WHEEL_MOTOR_PIN, OUTPUT);
    pinMode(LEFT_WHEEL_MOTOR_PIN, OUTPUT);
    pinMode(RPM_SENSOR_LEFT_WHEEL_PIN, INPUT);
    pinMode(RPM_SENSOR_RIGHT_WHEEL_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_LEFT_WHEEL_PIN), ISR_RpmSensorLeftWheel, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_RIGHT_WHEEL_PIN), ISR_RpmSensorRightWheel, RISING);
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_RIGHT_WHEEL_PIN), ISR_RpmSensorRightWheel, FALLING);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        RightMotor.attach(RIGHT_WHEEL_MOTOR_PIN, 1148, 1832);
        LeftMotor.attach(LEFT_WHEEL_MOTOR_PIN, 1148, 1832);
        RightMotor.write((int)90);
        LeftMotor.write((int)90);
    }
    
    myTimer.begin(power_train_sampling, SecToMicros(HzToSec(POWERTRAIN_PID_FREQUENCY_HZ)));  // blinkLED to run every 0.15 seconds
    
    Serial.begin(115200);
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
        pEnd = line.data();
        leftMotorRawSpeed = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        rightMotorRawSpeed = parseNextFloat(pEnd, (line.size() + line.data()) - pEnd, ';', &pEnd, &resultSuccess);
        line.clear();
    }

    temp_LeftWheelRpmData = getLeftWheelRpmData();
    temp_RightWheelRpmData = getRightWheelRpmData();
    //temp_RightWheelRpmData.on_pulse(&RightWheelRpmData);
    //Serial.println((long) temp_RightWheelRpmData.on_pulse);
    //Serial.println((long) on_pulse_right_motor);
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
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    powerTrain.SetRightWheelSpeedRequest_volatile(rightMotorRawSpeed);
    /*
    Serial.print("Right");
    Serial.print("\t");
    Serial.print("Raw Speed: ");
    Serial.print(powerTrain.GetRightWheelSpeedRequest_raw());
    Serial.print("\t");
    Serial.print("m/s: ");
    Serial.print(powerTrain.GetRightWheelSpeed());
    Serial.print("\t");
    Serial.print("Rpm: ");
    Serial.print(powerTrain.rightWheel.GetRpm());
    Serial.print("\t");
    Serial.print("TotalRotations: ");
    Serial.print(getRightWheelTotalRotations());
    Serial.println();
    */
    //power_train_sampling();
}
    delay(100);
}
