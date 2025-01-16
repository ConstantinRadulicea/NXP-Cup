#define DEBUG_UNITTEST
#define DEBUG_STEERINGWHEEL
#define _CRT_SECURE_NO_WARNINGS
#include "SteeringServo.h"
#include "SteeringWheel.h"
#include "PurePursuitGeometry.h"
#include "geometry2D.h"
#include <cstdint>
#include <vector>
#include <string.h>
#include <math.h>
#include <iostream>



#define WHEEL_BASE_M (0.176f)
#define WHEEL_DIAMETER_M (0.064f)	//wheel diameter im meters
#define DISTANCE_BETWEEN_WHEELS_M (0.137f)	//distance between wheels

#define STEERING_SERVO_ANGLE_MIDDLE     90
#define STEERING_SERVO_ANGLE_MAX_RIGHT  126   // +36 -> -36 going right 126
#define STEERING_SERVO_ANGLE_MAX_LEFT   48     // -47 -> +47 going left 43 //49
#define STEERING_SERVO_ERROR -4.0f


#define STEERING_SERVO_ANGLE_MAX_RIGHT  117   // -> -27 going right 126
#define STEERING_SERVO_ANGLE_MAX_LEFT   45     // -47 -> +45 going left 43 //49
#define STEERING_SERVO_ERROR -6.3f




int main333333333() {
	float arm_wheel_angle;
	float arm_wheel_length;
	arm_wheel_angle = 19.167f;
	arm_wheel_length = 24.5f;
	//arm_wheel_angle = 16.46f;
	//arm_wheel_length = 25.547f;
	SteeringWheel g_steering_wheel(WHEEL_BASE_M, DISTANCE_BETWEEN_WHEELS_M, STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, arm_wheel_angle, arm_wheel_length);
	g_steering_wheel.SetRawAngleOffset(STEERING_SERVO_ERROR);

	float servo_angle, servo_raw_angle, steering_wheel_angle, right_wheel_angle, left_wheel, right_wheel_achermann_angle, left_wheel_achermann_angle;
	float valid_angle;
	float right_wheel_error, left_wheel_error;
	printf("arm_wheel_angle:\t%.3f\n", arm_wheel_angle);
	printf("arm_wheel_length:\t%.3f\n", arm_wheel_length);
	printf("\n");
	printf("steering_angle\tvalid_steering_angle\tachermann_left_wheel\tachermann_right_wheel\tleft_wheel_angle\tright_wheel_angle\tleft_wheel_error\tright_wheel_error\tservo_angle\tservo_raw_angle");
	printf("\n");
	for (float i = -40; i <= 40; i += 0.1)
	{
		g_steering_wheel.SetRawAngleOffset(STEERING_SERVO_ERROR);
		//g_steering_wheel.setMaxLeftAngle_deg(40.0f);
		//g_steering_wheel.setMaxRightAngle_deg(-40.0f);

		valid_angle = g_steering_wheel.vaildAngleDeg(i);
		g_steering_wheel.setSteeringWheelAngleDeg(valid_angle);

		servo_angle = g_steering_wheel.steering_servo.getAngleDeg();
		servo_raw_angle = g_steering_wheel.steering_servo.getRawAngleDeg();
		steering_wheel_angle = g_steering_wheel.getSteeringWheelAngle();
		right_wheel_angle = g_steering_wheel.getRightWheelAngle_deg();
		left_wheel = g_steering_wheel.getLeftWheelAngle_deg();
		right_wheel_achermann_angle = g_steering_wheel.getRightWheelAchermannAngle_deg();
		left_wheel_achermann_angle = g_steering_wheel.getLeftWheelAchermannAngle_deg();
		right_wheel_error = right_wheel_achermann_angle - right_wheel_angle;
		left_wheel_error = left_wheel_achermann_angle - left_wheel;
		printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t", (float)i, valid_angle, left_wheel_achermann_angle, right_wheel_achermann_angle, left_wheel, right_wheel_angle, left_wheel_error, right_wheel_error, servo_angle, servo_raw_angle);
		printf("\n");
		//printf("req_ang [%d] = steer_wheel: %f\t left_wheel: %f\t right_wheel: %f\t servo: %f\t servo_raw: %f\n", i, steering_wheel_angle, left_wheel, right_wheel_angle, servo_angle, servo_raw_angle);
		//printf("Achermann:\tFL: %f\t\tFR: %f\n\n", left_wheel_achermann_angle, right_wheel_achermann_angle);
	}

	return 0;
}