#include <math.h>
#include "geometry2D.h"
#include <stdio.h>
#include <stdlib.h>

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
		this->desiredCarSpeed = 0;
		this->measuredSpeed = 0;
		this->timePassedFromLastSample = 0;
		this->speedRequest = 0;
		this->previous_error = 0;
		this->integral = 0;
	}

	void SetDesiredCarSpeed(float speed) {
		this->desiredCarSpeed = speed;
	}

	float GetDesiredCarSpeed() {
		return this->desiredCarSpeed;
	}

	float GetMeasuredSpeed() {
		return this->measuredSpeed;
	}

	//perioada de esantionare nu este constanta => timePassedFromLastSample1 din ISR
	void SetMeasuredSpeed(float measuredSpeed, volatile float timePassedFromLastSample1) {
		this->measuredSpeed = measuredSpeed;
		this->timePassedFromLastSample = timePassedFromLastSample1;
		this->speedRequest = CalculateSpeedRequest();
	}

	float GetSpeedRequest() {
		return this->speedRequest;
	}

	void SetToMotorRequestSpeed() {
		WriteToMotor(this->speedRequest);
	}

private:
	//carSpeed = calculateCarSpeed(), viteza dorita.
	float CalculateSpeedRequest() {
		float error = this->desiredCarSpeed - this->measuredSpeed;
		this->integral += error * this->timePassedFromLastSample;
		float derivative = (error - previous_error) / this->timePassedFromLastSample;
		float output_speed = (this->Kp * error) + (this->Ki * this->integral) + (this->Kd * derivative);
		previous_error = error;

		output_speed = fmax(0.0, fmin(output_speed, 180.0));
		return output_speed;
	}

	//=================================TO DO================================================
	void WriteToMotor(float speed) {
		printf("Motor set to speed %.2f (0-180) \n", speed);
	}
	//=================================TO DO================================================

	float desiredCarSpeed; //viteza calculata in raport cu camera
	float measuredSpeed; //viteza de la roti
	volatile float timePassedFromLastSample; //perioada de esantionare
	float speedRequest; //viteza sincronizata
	float previous_error;
	float integral;

	float Kp;
	float Ki;
	float Kd; 
};

class PowerTrain
{
public:
	PowerTrain(float kP, float kI, float kD, float wheelDiameter) : leftEngine(kP, kI, kD), rightEngine(kP, kI, kD), wheelDiameter(wheelDiameter){ }

	void SetSpeed(float speed, float leftWheelPercent, float rightWheelPercent) {
		if (leftWheelPercent < -1 || leftWheelPercent > 1 || rightWheelPercent < -1 || rightWheelPercent > 1) {
			printf("Error: wheel percent values must be in the range [-1, 1].\n");
			return;
		}
		
		float leftWheelSpeed = leftWheelPercent * speed;
		float rightWheelSpeed = rightWheelPercent * speed;

		float leftWheelSpeedRPM = MpsToRPM(leftWheelSpeed);
		float rightWheelSpeedRPM = MpsToRPM(rightWheelSpeed);

		leftEngine.SetDesiredCarSpeed(leftWheelSpeedRPM);
		rightEngine.SetDesiredCarSpeed(rightWheelSpeedRPM);
	}

	//==========================SIMULATION FUNCTION==============================
	void Simulate(float deltaTime, int cycles) {
		for (int i = 0; i < cycles; ++i) {
			float leftMeasuredSpeed = leftEngine.GetMeasuredSpeed();
			float rightMeasuredSpeed = rightEngine.GetMeasuredSpeed();

			leftEngine.SetMeasuredSpeed(leftMeasuredSpeed + (leftEngine.GetSpeedRequest() - leftMeasuredSpeed) * 0.1, deltaTime);
			rightEngine.SetMeasuredSpeed(rightMeasuredSpeed + (rightEngine.GetSpeedRequest() - rightMeasuredSpeed) * 0.1, deltaTime);

			printf("Cycle %d:\n", i);
			printf("  Left Engine - Desired: %.2f RPM, Measured: %.2f RPM\n", leftEngine.GetDesiredCarSpeed(), leftEngine.GetMeasuredSpeed());
			printf("  Right Engine - Desired: %.2f RPM, Measured: %.2f RPM\n", rightEngine.GetDesiredCarSpeed(), rightEngine.GetMeasuredSpeed());

			leftEngine.SetToMotorRequestSpeed();
			rightEngine.SetToMotorRequestSpeed();
		}
	}
	//==========================SIMULATION FUNCTION==============================

private:
	float MpsToRPM(float speedMps) { //metri/s --> RPM
		float wheelCircumference = M_PI * wheelDiameter; // wheel diameter is in meters!
		float speedRPM = (speedMps / wheelCircumference) * 60.0;
		
		return speedRPM;
	}

	EngineSync leftEngine;
	EngineSync rightEngine;
	float wheelDiameter;
};



int main() {
	float kP = 1.0;
	float kI = 0.1;
	float kD = 0.01;
	float wheelDiameter = 0.5; // exemplu: diametru roata in metri

	PowerTrain powerTrain(kP, kI, kD, wheelDiameter);

	// Exemplu de setare a vitezei
	float speed = 5.0; // viteza in metri/secunda
	float leftWheelPercent = 0.8; // procentul rotii stangi
	float rightWheelPercent = 0.7; // procentul rotii drepte

	powerTrain.SetSpeed(speed, leftWheelPercent, rightWheelPercent); // Setează viteza dorită pentru fiecare roată în funcție de procentajul specificat.

	float deltaTime = 0.1; // Perioada de eșantionare în secunde
	int cycles = 100; // Număr de cicluri de simulare

	powerTrain.Simulate(deltaTime, cycles);

	return 0;
}

