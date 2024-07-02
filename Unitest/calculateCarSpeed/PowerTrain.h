#ifndef __POWERTRAIN_H__
#define __POWERTRAIN_H__
#include <math.h>
//#include "geometry2D.h"


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2

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
#endif // DEBUG_UNIT_TEST


//TO DO:
//int read_encoder(int wheel_id); functie pentru encoder
//void WriteToMotor(float speed)

class EngineSync
{
public:
	EngineSync(float kP, float kI, float KD) {
		this->Kp = kP;
		this->Ki = kI;
		this->Kd = KD;
		this->speedRequest = 0.0;
		this->measuredSpeed = 0.0;
		this->timePassedFromLastSample = 0.0;
		this->speedRequest_raw = 0.0;
		this->previous_error = 0.0;
		this->integral = 0.0;
	}

	void SetSpeedRequest(float speed) {
		this->speedRequest = speed;
		this->SetToMotorSpeedRequest();
	}

	float GetSpeedRequest() {
		return this->speedRequest;
	}

	float GetMeasuredSpeed() {
		return this->measuredSpeed;
	}

	//perioada de esantionare nu este constanta => timePassedFromLastSample1 din ISR
	void SetMeasuredSpeed(float measuredSpeed, float timePassedFromLastSample1) {
		this->measuredSpeed = measuredSpeed;
		this->timePassedFromLastSample = timePassedFromLastSample1;
		this->SetToMotorSpeedRequest();
	}

	float GetSpeedRequest_raw() {
		return this->speedRequest_raw;
	}


private:
	void SetToMotorSpeedRequest() {
		this->speedRequest_raw = CalculateSpeedRequest_raw();
		WriteToMotor(this->speedRequest_raw);
	}
	//carSpeed = calculateCarSpeed(), viteza dorita.
	float CalculateSpeedRequest_raw() {
		float error = this->speedRequest - this->measuredSpeed;
		this->integral += error * this->timePassedFromLastSample;
		float derivative = (error - previous_error) / this->timePassedFromLastSample;
		float output_speed = (this->Kp * error) + (this->Ki * this->integral) + (this->Kd * derivative);
		previous_error = error;

		output_speed = fmax(0.0, fmin(output_speed, 180.0));
		return output_speed;
	}

	//=================================TO DO================================================
	virtual void WriteToMotor(float speed) {
		#ifdef DEBUG_UNIT_TEST
			printf("Motor set to speed %.2f (0-180) \n", speed);
		#endif // DEBUG_UNIT_TEST
	}
	//=================================TO DO================================================

	float speedRequest; //viteza calculata in raport cu camera
	float measuredSpeed; //viteza de la roti
	float timePassedFromLastSample; //perioada de esantionare
	float speedRequest_raw; //viteza sincronizata
	float previous_error;
	float integral;

	float Kp;
	float Ki;
	float Kd;
};

class PowerTrain
{
public:
	PowerTrain(float kP, float kI, float kD, float wheelDiameter_meters) : leftEngine(kP, kI, kD), rightEngine(kP, kI, kD), wheelDiameter(wheelDiameter_meters) {
	}

	void SetSpeed(float speed, float leftWheelPercent, float rightWheelPercent) {
		//if (leftWheelPercent < -1.0 || leftWheelPercent > 1.0 || rightWheelPercent < -1.0 || rightWheelPercent > 1.0) {
		//	printf("Error: wheel percent values must be in the range [-1, 1].\n");
		//	return;
		//}

		leftWheelPercent = MIN(1.0f, leftWheelPercent);
		leftWheelPercent = MAX(-1.0f, leftWheelPercent);

		rightWheelPercent = MIN(1.0f, rightWheelPercent);
		rightWheelPercent = MAX(-1.0f, rightWheelPercent);

		float leftWheelSpeed = leftWheelPercent * speed;
		float rightWheelSpeed = rightWheelPercent * speed;

		float leftWheelSpeedRPM = MpsToRPM(leftWheelSpeed);
		float rightWheelSpeedRPM = MpsToRPM(rightWheelSpeed);

		leftEngine.SetSpeedRequest(leftWheelSpeedRPM);
		rightEngine.SetSpeedRequest(rightWheelSpeedRPM);
	}
	float GetRightWheelSpeed() {
		return this->RPMToMps(this->rightEngine.GetMeasuredSpeed());
	}
	float GetLeftWheelSpeed() {
		return this->RPMToMps(this->leftEngine.GetMeasuredSpeed());
	}


#ifdef DEBUG_UNIT_TEST
	//==========================SIMULATION FUNCTION==============================
	void Simulate(float deltaTime, float cycles) {
		float leftMeasuredSpeed, rightMeasuredSpeed;
		// Parametrii funcției de transfer
		double mass = 1800.5; // kg
		double friction = 1.0; // N*s/m
		CarModel carModel(mass, friction, this->wheelDiameter / 2.0);
		for (int i = 0; i < (int)(cycles / deltaTime); ++i) {
			leftMeasuredSpeed = leftEngine.GetMeasuredSpeed();
			rightMeasuredSpeed = rightEngine.GetMeasuredSpeed();

			//leftEngine.SetMeasuredSpeed(leftMeasuredSpeed + (leftEngine.GetSpeedRequest_raw() - leftMeasuredSpeed) * 0.1, deltaTime);
			//rightEngine.SetMeasuredSpeed(rightMeasuredSpeed + (rightEngine.GetSpeedRequest_raw() - rightMeasuredSpeed) * 0.1, deltaTime);

			leftEngine.SetMeasuredSpeed(carModel.calculateRPM(leftEngine.GetSpeedRequest_raw(), i * deltaTime), deltaTime);
			rightEngine.SetMeasuredSpeed(carModel.calculateRPM(leftEngine.GetSpeedRequest_raw(), i * deltaTime), deltaTime);

			printf("Cycle %d:\n", i);
			printf("  Left Engine - Desired: %.2f RPM, Measured: %.2f RPM\n", leftEngine.GetSpeedRequest(), leftEngine.GetMeasuredSpeed());
			printf("  Right Engine - Desired: %.2f RPM, Measured: %.2f RPM\n", rightEngine.GetSpeedRequest(), rightEngine.GetMeasuredSpeed());

			//leftEngine.SetToMotorSpeedRequest();
			//rightEngine.SetToMotorSpeedRequest();
		}
	}
	//==========================SIMULATION FUNCTION==============================
#endif // DEBUG_UNIT_TEST



private:
	float MpsToRPM(float speedMps) { //metri/s --> RPM
		float wheelCircumference = M_PI * wheelDiameter; // wheel diameter is in meters!
		float speedRPM = (speedMps / wheelCircumference) * 60.0;

		return speedRPM;
	}

	float RPMToMps(float speedRPM) { //RPM --> metri/s
		return (((wheelDiameter / 2.0) * (2 * M_PI)) / 60.0) * speedRPM;
	}

	EngineSync leftEngine;
	EngineSync rightEngine;
	float wheelDiameter;
};

#endif // !__POWERTRAIN_H__
