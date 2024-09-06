#define DEBUG_UNIT_TEST
#include "PowerTrain.h"
#include "esc_raw.h"
#include <float.h>
#include <math.h>

// wheel diameter = 64mm

int PowerTrainfloatCmp(float num1, float num2) {
	if (fabs(num1 - num2) < FLT_EPSILON) {
		return 0;
	}
	else if (num1 > num2) {
		return 1;
	}
	return -1;
}

static float getRpmPulsePeriod_us(float Rpm) {
	float period;
	period = (1.0 / (Rpm / 60.0)) * 1000000.0;
	return period;
}

// left_right_turn: negative if turning left, positive if turning right
void SetSpeedRequest(float speed_ms, float turn_radius, int left_right_turn) {
	float left_wheel_turn_radius;
	float right_wheel_turn_radius;
	float left_wheel_turn_circonference;
	float right_wheel_turn_circonference;
	float car_trun_circonference;
	float left_wheel_speed_request_m;
	float right_wheel_speed_request_m;
	float _distanceBwWheels_m = 0.145;

	if (PowerTrainfloatCmp(turn_radius, 0.0) == 0 || left_right_turn == 0)	// going straight
	{
		left_wheel_speed_request_m = speed_ms;
		right_wheel_speed_request_m = speed_ms;
	}
	else {
		if (left_right_turn < 0)	// left turn
		{
			left_wheel_turn_radius = turn_radius - (_distanceBwWheels_m / 2.0);
			right_wheel_turn_radius = turn_radius + (_distanceBwWheels_m / 2.0);
		}
		else if (left_right_turn > 0)	// right turn
		{
			left_wheel_turn_radius = turn_radius + (_distanceBwWheels_m / 2.0);
			right_wheel_turn_radius = turn_radius - (_distanceBwWheels_m / 2.0);
		}

		left_wheel_turn_circonference = (2.0 * left_wheel_turn_radius) * M_PI;
		right_wheel_turn_circonference = (2.0 * right_wheel_turn_radius) * M_PI;
		car_trun_circonference = (2.0 * turn_radius) * M_PI;

		left_wheel_speed_request_m = (left_wheel_turn_circonference / car_trun_circonference) * speed_ms;
		right_wheel_speed_request_m = (right_wheel_turn_circonference / car_trun_circonference) * speed_ms;
	}

	//this->SetLeftWheelSpeedRequest(left_wheel_speed_request_m);
	//this->SetRightWheelSpeedRequest(right_wheel_speed_request_m);
}

int main() {
	float temp;
	float kP = 0.074;
	float kI = 0.1;
	float kD = 0.000;
	float wheelDiameter = 0.064; // exemplu: diametru roata in metri

	temp = getRpmPulsePeriod_us(300);
	temp = RpmToRaw(-1);
	temp = RpmToThrottle(300);

	SetSpeedRequest(1, 1, 1);

	PowerTrain g_powertrain(kP, kI, kD, wheelDiameter, NULL, NULL);
	g_powertrain.SetLeftWheelPID(kP, kI, kD, 1);

	g_powertrain.SetLeftWheelSpeedRequest(1.0);

	float deltaTime = 0.1; // Perioada de eșantionare în secunde
	int symulationTime_s = 1000; // Simulation time in seconds

	double val = 0.0;//g_powertrain.leftWheel.GetRpmRequest();
	double inc;
	double prev_err;
	double ggg = g_powertrain.leftWheel.GetRpmRequest();
	for (int i = 0; i < (int)(symulationTime_s / deltaTime); i++) {
		prev_err = g_powertrain.GetLeftWheelSpeedRequest_raw();
		g_powertrain.SetLeftWheelMeasuredRPM(val, deltaTime * 1000000);
		inc = g_powertrain.GetLeftWheelSpeedRequest_raw() - prev_err;
		printf("raw:%f m/s:%f\n", g_powertrain.GetLeftWheelSpeedRequest_raw(), g_powertrain.GetLeftWheelSpeed());
		val += (g_powertrain.GetLeftWheelSpeedRequest_raw());
		if (g_powertrain.GetLeftWheelSpeedRequest_raw() == 0.0f)
		{
			g_powertrain.leftWheel.GetRpmRequest();
		}
	}

	

	return 0;
}

