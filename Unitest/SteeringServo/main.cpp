
#define DEBUG_UNITTEST
#include "SteeringServo.h"
#include "SteeringWheel.h"

#include "PurePursuitGeometry.h"

#include "geometry2D.h"
#include <cstdint>
#include <vector>
#include <string.h>
#include <math.h>
#include <iostream>




// positive angle: going left
// negative angle: goinf right
// wheels angle: [59.711018; -48.946945], servo angle: [41; -65]
// servo angle: [60.33; -48.962376], wheel angle: [41.116337; -65.044365]
Point2D g_servo_position = Point2D{ 0.0f, 0.0f };
Point2D g_arm_wheel_position = Point2D{ 52.392, -6.0f };
float g_servo_arm_circle_radius_mm = 24.0f;	// 24mm, 20mm
float g_arm_wheel_circle_radius_mm = 25.547f;
float g_arm_wheel_angle_rad = NormalizePiToNegPi(radians(16.46f));		//16.46

// float g_servo_arm_to_arm_wheel_rod_length_mm = 45.6188202f;	// not necessary to calculate manually, already done automatically
float g_servo_rod_forward_angle_position_rad = NormalizePiToNegPi((M_PI_2 * 3.0f));
float g_arm_wheel_forward_angle_position_rad = NormalizePiToNegPi((M_PI_2 * 3.0f) - g_arm_wheel_angle_rad);



#define WHEEL_BASE_M 0.175
#define WHEEL_DIAMETER_M 0.064	//wheel diameter im meters
#define DISTANCE_BETWEEN_WHEELS_M 0.137	//distance between wheels



int main2() {
	float result_1, result_2, result_3;
	float servo, wheels, servo_2, right_wheel;

	SteeringConfiguration steer_config;
	steer_config.servo_arm_forward_position_angle_rad = g_servo_rod_forward_angle_position_rad;
	steer_config.servo_arm_length = g_servo_arm_circle_radius_mm;
	steer_config.servo_position = g_servo_position;
	steer_config.wheel_arm_forward_position_angle_rad = g_arm_wheel_forward_angle_position_rad;
	steer_config.wheel_arm_length = g_arm_wheel_circle_radius_mm;
	steer_config.wheel_position = g_arm_wheel_position;



	result_1 = degrees(WheelAngleToServoRawAngle_rad(radians(0), steer_config)); // -0.936093390 rad
	result_2 = degrees(ServoRawAngleToWheelAngle_rad(radians(result_1), steer_config)); // 40
	result_1 = degrees(WheelAngleToServoRawAngle_rad(radians(-40), steer_config)); // -0.936093390 rad
	result_2 = degrees(ServoRawAngleToWheelAngle_rad(radians(result_1), steer_config)); // -40
	for (int i = -50; i < 54; i++)
	{

		right_wheel = degrees(RightWheelAngle(radians(i), WHEEL_BASE_M, DISTANCE_BETWEEN_WHEELS_M));
		//right_wheel = i;
		servo_2 = degrees(WheelAngleToServoRawAngle_rad(radians(right_wheel), steer_config));
		wheels = degrees(ServoRawAngleToWheelAngle_rad(radians(servo_2), steer_config));
		printf("steering [%d] = wheel_RL: %f\t servo: %f\t wheels: %f\n", i, right_wheel, servo_2, wheels);
	}

	return 0;
}


#define STEERING_SERVO_ANGLE_MIDDLE     90
#define STEERING_SERVO_ANGLE_MAX_RIGHT  126   // +36 -> -36 going right
#define STEERING_SERVO_ANGLE_MAX_LEFT   43     // -47 -> +47 going left

int main() {
	SteeringWheel g_steering_wheel(WHEEL_BASE_M, DISTANCE_BETWEEN_WHEELS_M, STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);
	//g_steering_wheel.setTrackWidth(DISTANCE_BETWEEN_WHEELS_M);
	//g_steering_wheel.setWheelBase(WHEEL_BASE_M);
	//g_steering_wheel.steering_servo.setAngleOffset(-4.6f);

	float servo_angle, servo_raw_angle, steering_wheel_angle, right_wheel_angle;


	for (int i = -60; i <= 60; i++)
	{

		g_steering_wheel.setSteeringWheelAngleDeg(i);

		servo_angle = g_steering_wheel.steering_servo.getAngleDeg();
		servo_raw_angle = g_steering_wheel.steering_servo.getRawAngleDeg();
		steering_wheel_angle = g_steering_wheel.getSteeringWheelAngle();
		right_wheel_angle = g_steering_wheel.getRightWheelAngleDeg();
		printf("req_ang [%d] = steer_wheel: %f\t right_wheel: %f\t servo: %f\t servo_raw: %f\n", i, steering_wheel_angle, right_wheel_angle, servo_angle, servo_raw_angle);
	}
	return 0;
}