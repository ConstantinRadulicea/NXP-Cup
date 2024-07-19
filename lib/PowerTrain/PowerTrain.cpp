#include <Arduino.h>
#include "util/atomic.h"
#include <WheelRpm.h>
#include <PWMServo.h>
#include "PowerTrain.h"


#define POWERTRAIN_PID_FREQUENCY_HZ 100

IntervalTimer myTimer;

#define RPM_SENSOR_LEFT_WHEEL_PIN 3
#define RPM_SENSOR_RIGHT_WHEEL_PIN 4
#define RIGHT_WHEEL_MOTOR_PIN 23
#define LEFT_WHEEL_MOTOR_PIN 24

static float wheelDiameter = 0.064; // exemplu: diametru roata in metri

volatile PWMServo RightMotor;
volatile PWMServo LeftMotor;


void empty_function(){

}

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

    rpm = getCurrentRpm_adjusted(&temp_WheelRpmData);
    //timePassed = getRpmPulsePeriod_us(rpm);
    timePassed = getTimePassedFromLastSample_us_adjusted(&temp_WheelRpmData);
    powerTrain.SetRightWheelMeasuredRPM_volatile(rpm, timePassed);

    Serial.print(powerTrain.GetRightWheelSpeed());
    Serial.print(';');
    Serial.print(powerTrain.GetRightWheelSpeedRequest_raw());
    Serial.print(';');
    Serial.print(RightMotor.read());
    Serial.print(';');
    Serial.print(rpm);
    Serial.print(';');
    Serial.print(temp_WheelRpmData.Rpm);
    Serial.println();


    temp_WheelRpmData = getLeftWheelRpmData();
    rpm = getCurrentRpm_adjusted(&temp_WheelRpmData);
    //timePassed = getRpmPulsePeriod_us(rpm);
    timePassed = getTimePassedFromLastSample_us_adjusted(&temp_WheelRpmData);
    powerTrain.SetLeftWheelMeasuredRPM_volatile(rpm, timePassed);

}

void PowerTrainSetup(){
    float kp = 0.0;
    float ki = 3.0;
    float kd = 0.0;
    float ki_sum = 0.3;

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
}



