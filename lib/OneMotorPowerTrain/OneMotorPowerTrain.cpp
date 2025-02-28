#include "OneMotorPowerTrain.h"

static PWMServo _motor;
OneMotorPowerTrain g_onemotorpowertrain(0.0f, _motor);

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

static void MotorPinSetup(int motor_pin){
    if (motor_pin >= 0) {
        pinMode(motor_pin, OUTPUT);
        _motor.attach(motor_pin, 1148, 1832);
        _motor.write((int)90);
    }
}

static void WriteToMotorThrottle(float throttle){
    float rawValue;
    rawValue = 90.0 + (throttle * 90.0);
    rawValue = MAX(rawValue, 90.0);
    rawValue = MIN(rawValue, 180.0);
    _motor.write((int)rawValue);
    //Serial.println(String("%Right: ") + String(throttle, 2) + String(";") + String(rawValue));
}

void OneMotorPowerTrainSetup(float wheel_diameter_m, int motor_pin){
    g_onemotorpowertrain.SetDiameter(wheel_diameter_m);
    MotorPinSetup(motor_pin);
}



OneMotorPowerTrain::OneMotorPowerTrain(float wheelDiameter_meters_, PWMServo motor_) : motor(motor_) {
    this->wheelDiameter_meters = wheelDiameter_meters_;
}

void OneMotorPowerTrain::attach(int pin, int min, int max) {
    _motor.attach(pin, min, max);
}

void OneMotorPowerTrain::SetDiameter(float wheel_diameter) {
    this->wheelDiameter_meters = wheel_diameter;
}


void OneMotorPowerTrain::SetSpeedRequest(float speed_ms) {
    float throttle_request_temp;
    throttle_request_temp = RpmToThrottle(this->MpsToRPM(speed_ms));
    WriteToMotorThrottle(throttle_request_temp);
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

