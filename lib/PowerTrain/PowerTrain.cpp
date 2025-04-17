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

#include "PowerTrain.h"



static IntervalTimer myTimer;


volatile static PWMServo RightMotor;
volatile static PWMServo LeftMotor;
volatile static float pid_frequency_hz;

sampling_routine_callback sampl_routine_callback = NULL;


static void empty_function(){

}

static void WriteToMotor(volatile PWMServo* motor_, float angle_arg) {
    //ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        motor_->write((int)angle_arg);
    //}
}

static void WriteToRightMotor(float angle_arg){
    WriteToMotor(&RightMotor, angle_arg);
}

static void WriteToLeftMotor(float angle_arg){
    WriteToMotor(&LeftMotor, angle_arg);
}


static void WriteToRightMotorThrottle(float throttle){
    float rawValue;
    rawValue = 90.0 + (throttle * 90.0);
    rawValue = MAX(rawValue, 90.0);
    rawValue = MIN(rawValue, 180.0);
    WriteToRightMotor(rawValue);
    //Serial.println(String("%Right: ") + String(throttle, 2) + String(";") + String(rawValue));
}

static void WriteToLeftMotorThrottle(float throttle){
    float rawValue;
    rawValue = 90.0 + (throttle * 90.0);
    rawValue = MAX(rawValue, 90.0);
    rawValue = MIN(rawValue, 180.0);
    WriteToLeftMotor(rawValue);
    //Serial.println(String("%Left: ") + String(throttle, 2) + String(";") + String(rawValue));
}

volatile PowerTrain g_powertrain(0.0, 0.0, 0.0, 0.0, WriteToLeftMotorThrottle, WriteToRightMotorThrottle);


static void on_pulse_right_motor(volatile struct RpmSensorData *data){
}

static void on_pulse_left_motor(volatile struct RpmSensorData *data){
}



static void power_train_sampling(){
    float rpm, timePassed;
    RpmSensorData temp_WheelRpmData;

    temp_WheelRpmData = getRightWheelRpmData();

    rpm = getCurrentRpm_adjusted(&temp_WheelRpmData);
    //timePassed = getRpmPulsePeriod_us(rpm);
    timePassed = getTimePassedFromLastSample_us_adjusted(&temp_WheelRpmData);
    g_powertrain.SetRightWheelMeasuredRPM(rpm, timePassed);

    //Serial.print(g_powertrain.GetRightWheelSpeed());
    //Serial.print(';');
    //Serial.print(g_powertrain.GetRightWheelSpeedRequest_raw());
    //Serial.print(';');
    //Serial.print(RightMotor.read());
    //Serial.print(';');
    //Serial.print(rpm);
    //Serial.print(';');
    //Serial.print(temp_WheelRpmData.Rpm);
    //Serial.print(';');


    temp_WheelRpmData = getLeftWheelRpmData();
    rpm = getCurrentRpm_adjusted(&temp_WheelRpmData);
    //timePassed = getRpmPulsePeriod_us(rpm);
    timePassed = getTimePassedFromLastSample_us_adjusted(&temp_WheelRpmData);
    g_powertrain.SetLeftWheelMeasuredRPM(rpm, timePassed);

    //Serial.print(g_powertrain.GetLeftWheelSpeed());
    //Serial.print(';');
    //Serial.print(g_powertrain.GetLeftWheelSpeedRequest_raw());
    //Serial.print(';');
    //Serial.print(LeftMotor.read());
    //Serial.print(';');
    //Serial.print(rpm);
    //Serial.print(';');
    //Serial.print(temp_WheelRpmData.Rpm);
    //Serial.println();

    g_powertrain.SetSpeedRequest_slow_routine(HzToSec(pid_frequency_hz));

    if (sampl_routine_callback != NULL)
    {
        sampl_routine_callback();
    }
}

//int PowerTrainfloatCmp(float num1, float num2) {
//    return floatCmp(num1, num2);
//	if (fabs(num1 - num2) < FLT_EPSILON) {
//		return 0;
//	}
//	else if (num1 > num2) {
//		return 1;
//	}
//	return -1;
//}

static void MotorPinSetup(volatile PWMServo *servo_, int motor_pin){
    if (motor_pin >= 0) {
        pinMode(motor_pin, OUTPUT);
        servo_->attach(motor_pin, 1148, 1832);
        servo_->write((int)90);
    }
}


void PowerTrainSetup(float wheel_diameter_m, float distance_between_wheels_m, float _pid_frequency_hz, int left_motor_pin, int right_motor_pin, int left_rpm_sensor_pin, int right_rpm_sensor_pin, sampling_routine_callback sampl_routine_callback_)
{
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    float ki_sum = 0.0;

    sampl_routine_callback = sampl_routine_callback_;

    g_powertrain.SetRightWheelPID(kp, ki, kd, ki_sum);
    g_powertrain.SetRightWheelDiameter(wheel_diameter_m);
    g_powertrain.SetLeftWheelDiameter(wheel_diameter_m);
    g_powertrain.SetDistanceBetweenWheels(distance_between_wheels_m);

    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRightWheelRpmData();
    temp_WheelRpmData.on_pulse = on_pulse_right_motor;
    temp_WheelRpmData.PulsePin = right_rpm_sensor_pin;
    setRightWheelRpmData(temp_WheelRpmData);

    temp_WheelRpmData = getLeftWheelRpmData();
    temp_WheelRpmData.on_pulse = on_pulse_left_motor;
    temp_WheelRpmData.PulsePin = left_rpm_sensor_pin;
    setLeftWheelRpmData(temp_WheelRpmData);

    MotorPinSetup(&LeftMotor, left_motor_pin);
    MotorPinSetup(&RightMotor, right_motor_pin);

    if (left_rpm_sensor_pin >= 0) {
        pinMode(right_rpm_sensor_pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(left_rpm_sensor_pin), ISR_RpmSensorLeftWheel, RISING);
    }
    if (left_motor_pin >= 0) {
        pinMode(right_rpm_sensor_pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(right_rpm_sensor_pin), ISR_RpmSensorRightWheel, RISING);
    }
    
    g_powertrain.SetLeftWheelSpeedRequest(0.0);
    g_powertrain.SetRightWheelSpeedRequest(0.0);
    pid_frequency_hz = _pid_frequency_hz;
    myTimer.begin(power_train_sampling, SecToMicros(HzToSec(_pid_frequency_hz)));
}



