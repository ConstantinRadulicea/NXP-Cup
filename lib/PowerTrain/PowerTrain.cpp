#include "PowerTrain.h"


IntervalTimer myTimer;


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

volatile PowerTrain powerTrain(0.0, 0.0, 0.0, 0.0, WriteToLeftMotorThrottle, WriteToRightMotorThrottle);


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
    Serial.print(';');


    temp_WheelRpmData = getLeftWheelRpmData();
    rpm = getCurrentRpm_adjusted(&temp_WheelRpmData);
    //timePassed = getRpmPulsePeriod_us(rpm);
    timePassed = getTimePassedFromLastSample_us_adjusted(&temp_WheelRpmData);
    powerTrain.SetLeftWheelMeasuredRPM_volatile(rpm, timePassed);

    Serial.print(powerTrain.GetLeftWheelSpeed());
    Serial.print(';');
    Serial.print(powerTrain.GetLeftWheelSpeedRequest_raw());
    Serial.print(';');
    Serial.print(LeftMotor.read());
    Serial.print(';');
    Serial.print(rpm);
    Serial.print(';');
    Serial.print(temp_WheelRpmData.Rpm);
    Serial.println();
}

int PowerTrainfloatCmp(float num1, float num2) {
	if (fabs(num1 - num2) < FLT_EPSILON) {
		return 0;
	}
	else if (num1 > num2) {
		return 1;
	}
	return -1;
}


void PowerTrainSetup(float wheel_diameter_m, float distance_between_wheels_m, float pid_frequency_hz, int left_motor_pin, int right_motor_pin, int left_rpm_sensor_pin, int right_rpm_sensor_pin)
{
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    float ki_sum = 0.0;

    powerTrain.SetRightWheelPID(kp, ki, kd, ki_sum);
    powerTrain.SetRightWheelDiameter(wheel_diameter_m);
    powerTrain.SetLeftWheelDiameter(wheel_diameter_m);
    powerTrain.SetDistanceBetweenWheels(distance_between_wheels_m);

    RpmSensorData temp_WheelRpmData;
    temp_WheelRpmData = getRightWheelRpmData();
    temp_WheelRpmData.on_pulse = on_pulse_right_motor;
    temp_WheelRpmData.PulsePin = right_rpm_sensor_pin;
    setRightWheelRpmData(temp_WheelRpmData);

    temp_WheelRpmData = getLeftWheelRpmData();
    temp_WheelRpmData.on_pulse = on_pulse_left_motor;
    temp_WheelRpmData.PulsePin = left_rpm_sensor_pin;
    setLeftWheelRpmData(temp_WheelRpmData);

    if (left_motor_pin >= 0) {
        pinMode(left_motor_pin, OUTPUT);
        LeftMotor.attach(left_motor_pin, 1148, 1832);
        LeftMotor.write((int)90);
    }
    if (right_motor_pin >= 0) {
        pinMode(right_motor_pin, OUTPUT);
        RightMotor.attach(right_motor_pin, 1148, 1832);
        RightMotor.write((int)90);
    }
    if (left_rpm_sensor_pin >= 0) {
        pinMode(right_rpm_sensor_pin, INPUT_PULLDOWN);
        attachInterrupt(digitalPinToInterrupt(left_rpm_sensor_pin), ISR_RpmSensorLeftWheel, RISING);
    }
    if (left_motor_pin >= 0) {
        pinMode(right_rpm_sensor_pin, INPUT_PULLDOWN);
        attachInterrupt(digitalPinToInterrupt(right_rpm_sensor_pin), ISR_RpmSensorRightWheel, RISING);
    }
    
    powerTrain.SetLeftWheelSpeedRequest_volatile(0.0);
    powerTrain.SetRightWheelSpeedRequest_volatile(0.0);
    
    myTimer.begin(power_train_sampling, SecToMicros(HzToSec(pid_frequency_hz)));  // blinkLED to run every 0.15 seconds
}



