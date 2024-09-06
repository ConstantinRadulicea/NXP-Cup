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

#ifndef __POWERTRAIN_H__
#define __POWERTRAIN_H__

#include <Arduino.h>
#include <math.h>
#include <float.h>
#include "PID.h"
#include "esc_raw.h"
#include "util/atomic.h"
#include <WheelRpm.h>
#include <PWMServo.h>



#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define HzToSec(hz) (1.0/(hz))

#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2

int PowerTrainfloatCmp(float num1, float num2);


//TO DO:
//-int read_encoder(int wheel_id); functie pentru encoder
//-void WriteToMotor(float speed)
//-Chose a better rolling resistance coefficient!

#ifdef DEBUG_UNIT_TEST
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
class CarModel {
private:
	double mass;
	double friction;
	double radius;

public:
	CarModel(double mass_, double friction_, double wheel_radius) : mass(mass_), friction(friction_), radius(wheel_radius) {}

	double calculateRPM(double force, double time) {
		// Calculăm termenii funcției de transfer
		double K = 30 / (M_PI * radius);
		double a = mass;
		double b = friction;

		// Calculăm răspunsul în timp (time domain) pentru o intrare constantă (force)
		double exponent = -b / a * time;
		double omega = (K * force / b) * (1 - std::exp(exponent));

		return omega;
	}
};
#endif

typedef void (*MotorHandler) (float throttle);

class Wheel
{
public:
	Wheel(float kP_, float kI_, float kD_, float wheelDiameter_meters, MotorHandler motorHandler) {
		_pid = PID(kP_, kI_, kD_, -1.0, 1.0);
		this->wheelDiameter = wheelDiameter_meters;

		this->prevWheelRPM = 0.0;
		this->wheelRPM = 0.0;
		this->timePassedFromLastSample = 0.0;
		this->_motorHandler = motorHandler;
	}

	Wheel(float wheelDiameter_meters, MotorHandler motorHandler) {
		_pid = PID(0.0, 0.0, 0.0, -1.0, 1.0);

		this->wheelDiameter = wheelDiameter_meters;

		this->prevWheelRPM = 0.0;
		this->wheelRPM = 0.0;
		this->timePassedFromLastSample = 0.0;
		this->_motorHandler = motorHandler;
	}

	void SetMotorHandler(MotorHandler motorHandler) volatile {
		this->_motorHandler = motorHandler;
	}

	void SetPID(float kP, float kI, float kD, float integralImpact) volatile {
		_pid.setKp(kP);
		_pid.setKi(kI);
		_pid.setKd(kD);
		_pid.setIntegralImpact(integralImpact);
	}

	float GetSpeed() volatile{
		return this->RPMToMps(this->wheelRPM);
	}

	float GetRpm() volatile {
		return this->wheelRPM;
	}

	float GetRpmRequest() volatile {
		return this->MpsToRPM(this->GetSpeedRequest());
	}

	void SetToMotorRawRequest(float value) volatile {
		this->speedRequest_raw = value;
		if (this->_motorHandler != NULL) {
			this->_motorHandler(value);
		}
	}


	void SetSpeedRequest(float speed_mps) volatile{ // speed = mps!
		this->speedRequest = speed_mps;
		this->SetToMotorRawRequest(this->ThrottleToRawValue(this->CalculateThrottle(0.0)));
	}

	float GetSpeedRequest_raw() volatile {
		return this->speedRequest_raw;
	}

	float GetSpeedRequest()  volatile{
		return this->speedRequest;
	}

	float GetAcceleration() volatile {
		float accel = (RPMToMps(this->wheelRPM) - RPMToMps(this->prevWheelRPM)) / this->timePassedFromLastSample;
		return accel;
	}

	//perioada de esantionare nu este constanta => timePassedFromLastSample1 din ISR
	void SetMeasuredRPM(float measuredSpeed, float timePassedFromLastSample1) volatile {
		//Serial.println(measuredSpeed);
		//Serial.println(timePassedFromLastSample1);
		this->prevWheelRPM = this->wheelRPM;
		this->wheelRPM = measuredSpeed;
		this->timePassedFromLastSample = timePassedFromLastSample1;
		this->SetToMotorRawRequest(this->ThrottleToRawValue(this->CalculateThrottle(timePassedFromLastSample1)));
	}

	void SetDiameter(float wheel_diameter)  volatile {
		this->wheelDiameter = wheel_diameter;
	}
	float GetDiameter() volatile {
		return this->wheelDiameter;
	}

private:

	float ThrottleToRawValue(float value) volatile {
		value = MAX(0.0, value);
		value = MIN(1.0, value);
		//value = 90.0 + (value * 90.0);
		return value;
	}

	float MpsToRPM(float speedMps) volatile{ //metri/s --> RPM
		float wheelCircumference = M_PI * wheelDiameter; // wheel diameter is in meters!
		float speedRPM = (speedMps / wheelCircumference) * 60.0;

		return speedRPM;
	}

	float RPMToMps(float speedRPM) volatile { //RPM --> metri/s
		return (((wheelDiameter / 2.0) * (2 * M_PI)) / 60.0) * speedRPM;
	}


	float CalculateThrottle(double timePassedFromLastSample_) volatile{
		float throttle_request_temp;
		throttle_request_temp = RpmToThrottle(this->MpsToRPM(this->speedRequest));
		throttle_request_temp = throttle_request_temp + (float)this->_pid.calculate(this->speedRequest, this->RPMToMps(this->wheelRPM), timePassedFromLastSample_ / 1000000.0);
		return throttle_request_temp;
	}


	float prevWheelRPM = 0.0;
	float wheelRPM = 0.0;
	float wheelDiameter = 0.0;

	
	float speedRequest = 0.0;

	float carWeight = 0.0;
	float speedRequest_raw = 0.0;
	float timePassedFromLastSample = 0.0;
	MotorHandler _motorHandler = NULL;

	PID _pid;
};

class PowerTrain
{
public:

	PowerTrain(float kP, float kI, float kD, float wheelDiameter_meters, MotorHandler LeftMotorHandler, MotorHandler RightMotorHandler)
		:leftWheel(kP, kI, kD, wheelDiameter_meters, LeftMotorHandler),
		rightWheel(kP, kI, kD, wheelDiameter_meters, RightMotorHandler)
	{
	}

	PowerTrain(float wheelDiameter_meters, MotorHandler LeftMotorHandler, MotorHandler RightMotorHandler)
		:leftWheel(wheelDiameter_meters, LeftMotorHandler),
		rightWheel(wheelDiameter_meters, RightMotorHandler)
	{
	}

	void SetDistanceBetweenWheels(float distance_between_wheels) volatile {
		this->_distanceBwWheels_m = distance_between_wheels;
	}

	float GetDistanceBetweenWheels() volatile{
		return this->_distanceBwWheels_m;
	}

	// left_right_turn: negative if turning left, positive if turning right
	void SetSpeedRequest(float speed_ms, float turn_radius, int left_right_turn) volatile{
		float left_wheel_turn_radius;
		float right_wheel_turn_radius;
		float left_wheel_turn_circonference;
		float right_wheel_turn_circonference;
		float car_trun_circonference;
		float left_wheel_speed_request_m;
		float right_wheel_speed_request_m;

		turn_radius = fabs(turn_radius);

		if (PowerTrainfloatCmp(turn_radius, 0.0) == 0 || left_right_turn == 0)	// going straight
		{
			left_wheel_speed_request_m = speed_ms;
			right_wheel_speed_request_m = speed_ms;
		}
		else{
			if (left_right_turn < 0)	// left turn
			{
				left_wheel_turn_radius = turn_radius - (this->_distanceBwWheels_m / 2.0);
				right_wheel_turn_radius = turn_radius + (this->_distanceBwWheels_m / 2.0);
			}
			else if (left_right_turn > 0)	// right turn
			{
				left_wheel_turn_radius = turn_radius + (this->_distanceBwWheels_m / 2.0);
				right_wheel_turn_radius = turn_radius - (this->_distanceBwWheels_m / 2.0);
			}

			left_wheel_turn_circonference = (2.0 * left_wheel_turn_radius) * M_PI;
			right_wheel_turn_circonference = (2.0 * right_wheel_turn_radius) * M_PI;
			car_trun_circonference = (2.0 * turn_radius) * M_PI;
			
			left_wheel_speed_request_m =(left_wheel_turn_circonference / car_trun_circonference) * speed_ms;
			right_wheel_speed_request_m =(right_wheel_turn_circonference / car_trun_circonference) * speed_ms;
		}
		
		this->SetLeftWheelSpeedRequest(left_wheel_speed_request_m);
		this->SetRightWheelSpeedRequest(right_wheel_speed_request_m);
	}

	void SetLeftWheelMeasuredRPM(float measuredSpeed, float timePassedFromLastSample1) volatile {
		this->leftWheel.SetMeasuredRPM(measuredSpeed, timePassedFromLastSample1);
	}

	void SetRightWheelMeasuredRPM(float measuredSpeed, float timePassedFromLastSample1) volatile {
		this->rightWheel.SetMeasuredRPM(measuredSpeed, timePassedFromLastSample1);
	}


	void SetLeftWheelPID(float kP, float kI, float kD, float integralImpact) volatile {
		this->leftWheel.SetPID(kP, kI, kD, integralImpact);
	}
	void SetRightWheelPID(float kP, float kI, float kD, float integralImpact) volatile {
		this->rightWheel.SetPID(kP, kI, kD, integralImpact);
	}

	void SetRightWheelDiameter(float wheel_diameter) volatile {
		this->rightWheel.SetDiameter(wheel_diameter);
	}

	void SetLeftWheelDiameter(float wheel_diameter) volatile {
		this->leftWheel.SetDiameter(wheel_diameter);
	}

	float GetRightWheelSpeed() volatile {
		return this->rightWheel.GetSpeed();
	}
	float GetLeftWheelSpeed() volatile {
		return this->leftWheel.GetSpeed();
	}

	void SetRightWheelSpeedRequest(float speed_mps) volatile {
		this->rightWheel.SetSpeedRequest(speed_mps);
	}
	void SetLeftWheelSpeedRequest(float speed_mps) volatile {
		this->leftWheel.SetSpeedRequest(speed_mps);
	}


	float GetRightWheelSpeedRequest_raw() volatile {
		return this->rightWheel.GetSpeedRequest_raw();
	}
	float GetLeftWheelSpeedRequest_raw()  volatile{
		return this->leftWheel.GetSpeedRequest_raw();
	}

	float GetRightWheelSpeedRequest() volatile {
		return this->rightWheel.GetSpeedRequest();
	}
	float GetLeftWheelSpeedRequest() volatile {
		return this->leftWheel.GetSpeedRequest();
	}


	float GetLeftWheelAcceleration() volatile {
		return this->leftWheel.GetAcceleration();
	}
	float GetRightWheelAcceleration() volatile {
		return this->rightWheel.GetAcceleration();
	}
	/*
	void SetRightWheelAccelerationRequest(float speed_mps2) {
		this->rightWheel.SetAccelerationRequest(speed_mps2);
	}
	void SetLeftWheelAccelerationRequest(float speed_mps2) {
		this->leftWheel.SetAccelerationRequest(speed_mps2);
	}
	*/
	Wheel leftWheel;
	Wheel rightWheel;
	float _distanceBwWheels_m = 0.0;

private:
};

void PowerTrainSetup(float wheel_diameter_m, float distance_between_wheels_m, float pid_frequency_hz, int left_motor_pin, int right_motor_pin, int left_rpm_sensor_pin, int right_rpm_sensor_pin);
extern volatile PowerTrain g_powertrain;


#endif // !__POWERTRAIN_H__
