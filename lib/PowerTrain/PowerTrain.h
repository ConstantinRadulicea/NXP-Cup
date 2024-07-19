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

#include <math.h>
#include "PID.h"
#include "esc_raw.h"


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define HzToSec(hz) (1.0/(hz))


#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2


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

	void SetMotorHandler(MotorHandler motorHandler){
		this->_motorHandler = motorHandler;
	}

	void SetPID(float kP, float kI, float kD, float integralImpact) {
		_pid.setKp(kP);
		_pid.setKi(kI);
		_pid.setKd(kD);
		_pid.setIntegralImpact(integralImpact);
	}

	float GetSpeed() {
		return this->RPMToMps(this->wheelRPM);
	}

	float GetRpm() {
		return this->wheelRPM;
	}

	float GetRpmRequest() {
		return this->MpsToRPM(this->GetSpeedRequest());
	}

	void SetToMotorRawRequest(float value) {
		this->speedRequest_raw = value;
		if (this->_motorHandler != NULL) {
			this->_motorHandler(value);
		}
	}


	void SetSpeedRequest(float speed_mps) { // speed = mps!
		this->speedRequest = speed_mps;
		this->SetToMotorRawRequest(this->ThrottleToRawValue(this->CalculateThrottle(0.0)));
	}

	float GetSpeedRequest_raw() {
		return this->speedRequest_raw;
	}

	float GetSpeedRequest() {
		return this->speedRequest;
	}

	float GetAcceleration() {
		float accel = (RPMToMps(this->wheelRPM) - RPMToMps(this->prevWheelRPM)) / this->timePassedFromLastSample;
		return accel;
	}

	//perioada de esantionare nu este constanta => timePassedFromLastSample1 din ISR
	void SetMeasuredRPM(float measuredSpeed, float timePassedFromLastSample1) {
		//Serial.println(measuredSpeed);
		//Serial.println(timePassedFromLastSample1);
		this->prevWheelRPM = this->wheelRPM;
		this->wheelRPM = measuredSpeed;
		this->timePassedFromLastSample = timePassedFromLastSample1;
		this->SetToMotorRawRequest(this->ThrottleToRawValue(this->CalculateThrottle(timePassedFromLastSample1)));
	}
	float timePassedFromLastSample = 0.0;

private:

	float ThrottleToRawValue(float value) {
		//value = 90.0 + (value * 90.0);
		return value;
	}

	float MpsToRPM(float speedMps) { //metri/s --> RPM
		float wheelCircumference = M_PI * wheelDiameter; // wheel diameter is in meters!
		float speedRPM = (speedMps / wheelCircumference) * 60.0;

		return speedRPM;
	}

	float RPMToMps(float speedRPM) { //RPM --> metri/s
		return (((wheelDiameter / 2.0) * (2 * M_PI)) / 60.0) * speedRPM;
	}


	float CalculateThrottle(double timePassedFromLastSample_) {
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
	float speedRequest_raw = 90.0;
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

	void SetLeftWheelMeasuredRPM(float measuredSpeed, float timePassedFromLastSample1) {
		this->leftWheel.SetMeasuredRPM(measuredSpeed, timePassedFromLastSample1);
	}

	void SetRightWheelMeasuredRPM(float measuredSpeed, float timePassedFromLastSample1) {
		this->rightWheel.SetMeasuredRPM(measuredSpeed, timePassedFromLastSample1);
	}




	void SetLeftWheelMeasuredRPM_volatile(float measuredSpeed, float timePassedFromLastSample1) {
		this->SetLeftWheelMeasuredRPM(measuredSpeed, timePassedFromLastSample1);
	}

	void SetRightWheelMeasuredRPM_volatile(float measuredSpeed, float timePassedFromLastSample1) {
		this->SetRightWheelMeasuredRPM(measuredSpeed, timePassedFromLastSample1);
	}




	void SetLeftWheelPID(float kP, float kI, float kD, float integralImpact) {
		this->leftWheel.SetPID(kP, kI, kD, integralImpact);
	}
	void SetRightWheelPID(float kP, float kI, float kD, float integralImpact) {
		this->rightWheel.SetPID(kP, kI, kD, integralImpact);
	}

	float GetRightWheelSpeed() volatile {
		return this->rightWheel.GetSpeed();
	}
	float GetLeftWheelSpeed() {
		return this->leftWheel.GetSpeed();
	}

	void SetRightWheelSpeedRequest(float speed_mps) {
		this->rightWheel.SetSpeedRequest(speed_mps);
	}
	void SetLeftWheelSpeedRequest(float speed_mps) {
		this->leftWheel.SetSpeedRequest(speed_mps);
	}


	void SetRightWheelSpeedRequest_volatile(float speed_mps) volatile{
		this->SetRightWheelSpeedRequest(speed_mps);
	}
	void SetLeftWheelSpeedRequest_volatile(float speed_mps) volatile {
		this->SetLeftWheelSpeedRequest(speed_mps);
	}



	float GetRightWheelSpeedRequest_raw() volatile {
		return this->rightWheel.GetSpeedRequest_raw();
	}
	float GetLeftWheelSpeedRequest_raw() {
		return this->leftWheel.GetSpeedRequest_raw();
	}

	float GetRightWheelSpeedRequest() {
		return this->rightWheel.GetSpeedRequest();
	}
	float GetLeftWheelSpeedRequest() {
		return this->leftWheel.GetSpeedRequest();
	}


	float GetLeftWheelAcceleration() {
		return this->leftWheel.GetAcceleration();
	}
	float GetRightWheelAcceleration() {
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

private:
};

void PowerTrainSetup();
extern volatile PowerTrain powerTrain;

#endif // !__POWERTRAIN_H__
