#define DEBUG_UNIT_TEST
#include "PowerTrain.h"

// wheel diameter = 64mm

int main() {
	float kP = 0.001;
	float kI = 0.01;
	float kD = 0.001;
	float wheelDiameter = 0.064; // exemplu: diametru roata in metri

	PowerTrain powerTrain(kP, kI, kD, wheelDiameter);

	powerTrain.SetLeftWheelSpeedRequest(1.0);

	float deltaTime = 0.1; // Perioada de eșantionare în secunde
	int symulationTime_s = 100; // Simulation time in seconds

	double val = 0.0;//powerTrain.leftWheel.GetRpmRequest();
	double inc;
	double prev_err;
	double ggg = powerTrain.leftWheel.GetRpmRequest();
	for (int i = 0; i < (int)(symulationTime_s / deltaTime); i++) {
		prev_err = powerTrain.GetLeftWheelSpeedRequest_raw();
		powerTrain.SetLeftWheelMeasuredRPM(val, deltaTime);
		inc = powerTrain.GetLeftWheelSpeedRequest_raw() - prev_err;
		printf("raw:%f m/s:%f\n", powerTrain.GetLeftWheelSpeedRequest_raw(), powerTrain.GetLeftWheelSpeed());
		val += (powerTrain.GetLeftWheelSpeedRequest_raw() - 90);
		if (powerTrain.GetLeftWheelSpeedRequest_raw() == 0.0f)
		{
			powerTrain.leftWheel.GetRpmRequest();
		}
	}

	return 0;
}

