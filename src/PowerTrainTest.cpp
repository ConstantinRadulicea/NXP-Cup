#include <Arduino.h>
#include <util/atomic.h>
#include "WheelRpm.h"
#include "Config.h"
#include "ReadSerial.h"
#include "PowerTrain.h"
#include <vector>
#include "MovingAverage.h"


#define POWERTRAIN_PID_FREQUENCY_HZ 100
#define HzToSec(hz) (1.0/(hz))

IntervalTimer myTimer;

#define RPM_SENSOR_LEFT_WHEEL_PIN 3
#define RPM_SENSOR_RIGHT_WHEEL_PIN 4
#define RIGHT_WHEEL_MOTOR_PIN 23
#define LEFT_WHEEL_MOTOR_PIN 24

float wheelDiameter = 0.064; // exemplu: diametru roata in metri

volatile PWMServo RightMotor;
volatile PWMServo LeftMotor;

float kp = 0.075, ki = 0.0, kd = 0.0, ki_sum = 0.0;

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
    rawValue = MIN(rawValue, 180.0);
    WriteToRightMotor(rawValue);
}

void WriteToLeftMotorThrottle(float throttle){
    float rawValue;
    rawValue = 90.0 + (throttle * 90.0);
    rawValue = MAX(rawValue, 90.0);
    rawValue = MIN(rawValue, 180.0);
    WriteToLeftMotor(rawValue);
}

volatile PowerTrain powerTrain(0.0, 0.0, 0.0, wheelDiameter, WriteToLeftMotorThrottle, WriteToRightMotorThrottle);


void on_pulse_right_motor(volatile struct RpmSensorData *data){
}

void on_pulse_left_motor(volatile struct RpmSensorData *data){
}


void power_train_sampling(){
    float rpm, timePassed;
    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRightWheelRpmData();

    rpm = temp_WheelRpmData.RpmFiltered;
    timePassed = temp_WheelRpmData.TimePassedFromLastSample_us;
    if (getTimePassedFromLastSample_us_adjusted(&RightWheelRpmData) > (3.0*timePassed)) {
        timePassed = getTimePassedFromLastSample_us_adjusted(&RightWheelRpmData);
        rpm = getRpmFiltered_adjusted(&RightWheelRpmData);
    }

    
    if (getTimePassedFromLastSample_us_adjusted(&RightWheelRpmData) > MillisToMicros(2000)) {
        rpm = 0.0;
    }
    powerTrain.SetRightWheelMeasuredRPM_volatile(rpm, timePassed);

    //Serial.print(powerTrain.GetRightWheelSpeed());
    //Serial.print(';');
    //Serial.print(powerTrain.GetRightWheelSpeedRequest_raw());
    //Serial.print(';');
    //Serial.print(RightMotor.read());
    //Serial.print(';');
    Serial.print(rpm);
    Serial.println();


    temp_WheelRpmData = getLeftWheelRpmData();
    rpm = temp_WheelRpmData.RpmFiltered;
    timePassed = temp_WheelRpmData.TimePassedFromLastSample_us;
    if (getTimePassedFromLastSample_us_adjusted(&LeftWheelRpmData) > MillisToMicros(1000)) {
        rpm = 0.0;
    }
    
    powerTrain.SetLeftWheelMeasuredRPM_volatile(rpm, timePassed);

}
void empty_function(){

}

void setup() {
    powerTrain.SetRightWheelPID(kp, ki, kd, ki_sum);
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
    powerTrain.SetLeftWheelSpeedRequest_volatile(0.0);
    powerTrain.SetRightWheelSpeedRequest_volatile(0.0);
    
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
    //powerTrain.SetRightWheelPID(kp, ki, kd, ki_sum);
    //powerTrain.SetRightWheelSpeedRequest_volatile(rightMotorRawSpeed);
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
