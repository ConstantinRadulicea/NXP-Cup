#include "OneMotorPowerTrain.h"

OneMotorPowerTrain g_onemotorpowertrain(0.0f);

void OneMotorPowerTrainSetup(float wheel_diameter_m, int motor_pin){
    g_onemotorpowertrain.SetDiameter(wheel_diameter_m);
    if (motor_pin >= 0) {
        pinMode(motor_pin, OUTPUT);
        g_onemotorpowertrain.attach(motor_pin, 1148, 1832);
        g_onemotorpowertrain.SetSpeedRequest(0.0f);
    }
}



OneMotorPowerTrain::OneMotorPowerTrain(float wheelDiameter_meters_) {
    this->wheelDiameter_meters = wheelDiameter_meters_;
}

void OneMotorPowerTrain::attach(int pin, int min, int max) {
    this->motor.attach(pin, min, max);
}

void OneMotorPowerTrain::SetDiameter(float wheel_diameter) {
    this->wheelDiameter_meters = wheel_diameter;
}


void OneMotorPowerTrain::SetSpeedRequest(float speed_ms) {
    float throttle_request_temp;
    throttle_request_temp = RpmToThrottle(this->MpsToRPM(speed_ms));
    this->motor.write(throttle_request_temp);
}

void OneMotorPowerTrain::SetSpeedRequest_slow(float speed_ms) {
    this->SetSpeedRequest(speed_ms);
}

	float OneMotorPowerTrain::MpsToRPM(float speedMps) { //metri/s --> RPM
		float wheelCircumference = M_PI * this->wheelDiameter_meters; // wheel diameter is in meters!
		float speedRPM = (speedMps / wheelCircumference) * 60.0;

		return speedRPM;
	}


OneMotorPowerTrain::~OneMotorPowerTrain()
{
}

