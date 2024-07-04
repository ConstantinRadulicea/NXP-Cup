#ifndef __POWERTRAIN_H__
#define __POWERTRAIN_H__
#include <math.h>
//#include "geometry2D.h"


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


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
#endif // DEBUG_UNIT_TEST

float CalculateTorque(float mass, float speed_ms, float radius) {
	const float gForce = 9.81; //Acceleration due to gravity (m/s^2)
	const float C_rr = 0.015; //rolling resistance coefficient/coeficientul de frecare
	
	float fRolling = C_rr * mass * gForce;
	float pRolling = fRolling * speed_ms;

	float acceleration;
	float pAcceleration = 0.5 * mass * acceleration * speed_ms;

	float pTotal = pRolling + pAcceleration;

	float angularVelocity = speed_ms / radius;

	float torque = pTotal / angularVelocity;
	return torque;
}

class Engine
{
public:
	Engine(float batteryAmperage, float batteryVoltage, float motorKV) {
		this->torqueRequest = 0.0;
		this->speedRequest_raw = 0.0;

		this->batteryAmperage = batteryAmperage = 2.2;
		this->batteryVoltage = batteryVoltage = 7.4;
		this->motorKV = motorKV= 930.0;
		this->maxTorque = (60.0 * this->batteryAmperage) / (this->motorKV * 2.0 * M_PI); // Torque max = 0.0226 Nm
	}

	void SetTorqueRequest(float torque) {
		this->torqueRequest = torque;
		this->SetToMotorRawRequest(this->TorqueToRawValue(torque));
	}

	float GetTorqueRequest() {
		return this->torqueRequest;
	}

	float GetSpeedRequest_raw() {
		return this->speedRequest_raw;
	}



protected:
	
	float TorqueToRawValue(float torque) {
		float rawValue;

		rawValue = torque / this->maxTorque;
		rawValue = (rawValue * 90.0) + 90.0;

		return rawValue;
	}

	float RPMtoTorque(float RPM, float voltage, float current) {
		float angularSpeed = (RPM * 2 * M_PI) / 60.0; //rad/s

		float power = voltage * current; //wats

		float torqueFromPower = power / angularSpeed; //Nm (Newton-metri)

		return torqueFromPower;
	}

	float TorqueToRPM(float torque, float voltage, float current) {
		// Calculate power in Watts
		float power = voltage * current;

		// Calculate angular speed in rad/s using torque
		float angularSpeed = power / torque; // rad/s

		// Convert angular speed to RPM
		float RPM = (angularSpeed * 60.0) / (2 * M_PI); // RPM

		return RPM;

	}

	void SetToMotorRawRequest(float value) {
		this->speedRequest_raw = value;
		WriteToMotor(value);
	}

	//=================================TO DO================================================
	virtual void WriteToMotor(float speed) {
		#ifdef DEBUG_UNIT_TEST
			printf("Motor set to speed %.2f (0-180) \n", speed);
		#endif // DEBUG_UNIT_TEST
	}
	//=================================TO DO================================================

	float torqueRequest; 

	float motorKV;
	float batteryVoltage;
	float batteryAmperage;
	float speedRequest_raw;

	float maxTorque;

	float Kp;
	float Ki;
	float Kd;
};

class Wheel: protected Engine //read RPM and set torque
{
public:
	Wheel(float kP, float kI, float kD, float wheelDiameter_meters, float batteryAmperage, float batteryVoltage, float motorKV): Wheel::Engine(batteryAmperage, batteryVoltage, motorKV), wheelDiameter(wheelDiameter_meters) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
	}
	
	float GetSpeed() {
		return this->RPMToMps(this->wheelRPM);
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

	void SetSpeed(float speed_mps) { // speed = mps!
		this->speedRequest = speed_mps;
		this->SetTorqueRequest(this->CalculateTorqueRequest());
	}

	//perioada de esantionare nu este constanta => timePassedFromLastSample1 din ISR
	void SetMeasuredRPM(float measuredSpeed, float timePassedFromLastSample1) {
		this->wheelRPM = measuredSpeed;
		this->timePassedFromLastSample = timePassedFromLastSample1;
		this->SetTorqueRequest(this->CalculateTorqueRequest());
	}

private:
	float MpsToRPM(float speedMps) { //metri/s --> RPM
		float wheelCircumference = M_PI * wheelDiameter; // wheel diameter is in meters!
		float speedRPM = (speedMps / wheelCircumference) * 60.0;

		return speedRPM;
	}

	float RPMToMps(float speedRPM) { //RPM --> metri/s
		return (((wheelDiameter / 2.0) * (2 * M_PI)) / 60.0) * speedRPM;
	}

	//carSpeed = calculateCarSpeed(), viteza dorita.
	float CalculateTorqueRequest() { 
		float error = this->speedRequest - this->RPMToMps(this->wheelRPM);
		this->integral += error * this->timePassedFromLastSample;
		float derivative = (error - previous_error) / this->timePassedFromLastSample;
		float output_speed = (this->Kp * error) + (this->Ki * this->integral) + (this->Kd * derivative);
		previous_error = error;

		output_speed = fmax(0.0, fmin(output_speed, 180.0));
		return output_speed; 
	}

	float wheelRPM;
	float wheelDiameter;

	float timePassedFromLastSample;
	float speedRequest;
	float previous_error;
	float integral;

	float carWeight;
};

class PowerTrain
{
public:

	PowerTrain(float kP, float kI, float kD, float wheelDiameter_meters, float batteryAmperage, float batteryVoltage, float motorKV)
		:leftWheel(kP, kI, kD, wheelDiameter_meters, batteryAmperage, batteryVoltage, motorKV), 
		rightWheel(kP, kI, kD, wheelDiameter_meters, batteryAmperage, batteryVoltage, motorKV) { }
	
	float GetRightWheelSpeed() {
		return this->leftWheel.GetSpeed();
	}
	float GetLeftWheelSpeed() {
		return this->rightWheel.GetSpeed();
	}

	void SetRightWheelSpeed(float speed_mps) {
		this->rightWheel.SetSpeed(speed_mps);
	}
	void SetLeftWheelSpeed(float speed_mps) {
		this->leftWheel.SetSpeed(speed_mps);
	}


private:
	
	float carWeight;
	Wheel leftWheel;
	Wheel rightWheel;

};



#endif // !__POWERTRAIN_H__
