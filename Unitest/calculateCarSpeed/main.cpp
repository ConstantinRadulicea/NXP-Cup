#include <math.h>
#include "geometry2D.h"
#include <stdio.h>
#include <stdlib.h>

//TO DO:
//int read_encoder(int wheel_id); functie de citire senzorul de rotatii/viteza
void set_motor_speed(int motor_id, double carSpeed) {
	printf("Motor %d set to speed %.2f (0-180) \n", motor_id, carSpeed);
}


class EnginesSync 
{
private:
	double Kp;
	double Ki;
	double Kd;

	long previousTime = 0;
	double previous_error1;
	double integral1;

public:

	void init_pid(double kp, double ki, double kd) {
		this->Kp = kp;
		this->Ki = ki;
		this->Kd = kd;

		this->previous_error1 = 0;
		this->integral1 = 0;
	}
	
	//carSpeed = calculateCarSpeed(), viteza dorita.
	double sync_motors(double carSpeed, double measured_speed1) {
		//long currentTime = micros();
		//float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;

		double error1 = carSpeed - measured_speed1;
		//integral1 = integral1 + error1 * deltaT;
		integral1 = integral1 + error1;
		//double derivative1 = (error1 - previous_error1) / deltaT;
		double derivative1 = error1 - previous_error1;
		//previousTime = currentTime;
		previous_error1 = error1;

		double output_speed2 = (this->Kp * error1) + (this->Ki * integral1) + (this->Kd * derivative1);
		output_speed2 = fmax(0.0, fmin(output_speed2, 180.0));

		set_motor_speed(2, output_speed2);//unul dintre motoare va fi Master iar al doilea Slave.

		return output_speed2;
	}

};


int main() {
	EnginesSync engines;
	engines.init_pid(1.0, 0.05, 0.1);

	// Viteze dorite pentru fiecare motor (0 - 180)
	double carSpeed1 = 120.0;

	// Viteza măsurată pentru simulație (0 - 180)
	double measured_speed1 = 120.0;

	for (int i = 0; i < 500; ++i) { 
		double output_speed2 = engines.sync_motors(carSpeed1, measured_speed1);
		// Simulăm măsurătoarea reală printr-o interpolare simplă
		measured_speed1 += 0.1 * (output_speed2 - measured_speed1);
	}

	return 0;
}

