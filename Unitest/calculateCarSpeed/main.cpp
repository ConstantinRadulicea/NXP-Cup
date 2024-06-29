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
	}

	float GetSpeedRequest() {
		this->speedRequest = CalculateSpeedRequest();
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
	volatile float timePassedFromLastSample = 0; //perioada de esantionare
	float speedRequest; //viteza sincronizata
	float previous_error = 0;
	float integral = 0;

	float Kp;
	float Ki;
	float Kd; 
};


int main() {
	EngineSync engine(1.0, 0.1, 0.01);

	//Viteze(0 - 180)
	engine.SetDesiredCarSpeed(120.0);
	engine.SetMeasuredSpeed(135.0, 1.0);
	float output_speed;

	for (int i = 0; i < 500; ++i) { 
		output_speed = engine.GetSpeedRequest();
		engine.SetToMotorRequestSpeed();

		// Simulare: actualizare viteza masurata (într-o situație reală, aceasta ar veni de la senzori)
		float newMeasuredSpeed = engine.GetMeasuredSpeed() + (output_speed - engine.GetMeasuredSpeed()) * 0.1;
		engine.SetMeasuredSpeed(newMeasuredSpeed, 1.0);

		printf("Measured Speed: %.2f\n", engine.GetMeasuredSpeed());
	}

	return 0;
}

