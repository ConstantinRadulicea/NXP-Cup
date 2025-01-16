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




// positive angle: going left
// negative angle: goinf right
// wheels angle: [59.711018; -48.946945], servo angle: [41; -65]
// servo angle: [60.33; -48.962376], wheel angle: [41.116337; -65.044365]
//arm_wheel_rod_position{ x = 44.7613602 y = -30.9000263 }	Point2D

Point2D g_servo_position = Point2D{ 0.0f, 0.0f };
Point2D g_arm_wheel_position = Point2D{ 52.0f, -6.4f };//Point2D{ 52.392, -6.0f };
float g_servo_arm_circle_radius_mm = 24.0f;	// 24mm, 20mm
float g_arm_wheel_circle_radius_mm = 25.547f;
float g_arm_wheel_angle_rad = NormalizePiToNegPi(radians(16.46f));		//16.46

float g_servo_arm_forward_angle_position_rad = NormalizePiToNegPi((M_PI_2 * 3.0f));
float g_wheel_arm_forward_angle_position_rad = NormalizePiToNegPi((M_PI_2 * 3.0f) - g_arm_wheel_angle_rad);



#define WHEEL_BASE_M (0.176f)
#define WHEEL_DIAMETER_M (0.064f)	//wheel diameter im meters
#define DISTANCE_BETWEEN_WHEELS_M (0.137f)	//distance between wheels





int main2() {
	float result_1, result_2, result_3;
	float servo, wheels, servo_2, right_wheel, left_wheel, left_wheel_angle_from_steering, right_wheel_angle_from_steering;
	float trackwidth;
	float steering_angle = 0.0f;
	left_wheel = 0.0f;
	wheels = 0.0f;

	SteeringConfiguration right_wheel_config;
	right_wheel_config.servo_arm_forward_position_angle_rad = g_servo_arm_forward_angle_position_rad;
	right_wheel_config.servo_arm_length = g_servo_arm_circle_radius_mm;
	right_wheel_config.servo_position = g_servo_position;
	right_wheel_config.wheel_arm_forward_position_angle_rad = g_wheel_arm_forward_angle_position_rad;
	right_wheel_config.wheel_arm_length = g_arm_wheel_circle_radius_mm;
	right_wheel_config.wheel_position = g_arm_wheel_position;

	SteeringConfiguration left_wheel_config;
	left_wheel_config.servo_arm_forward_position_angle_rad = g_wheel_arm_forward_angle_position_rad;
	left_wheel_config.servo_arm_length = g_arm_wheel_circle_radius_mm;
	left_wheel_config.servo_position = g_arm_wheel_position;
	left_wheel_config.wheel_arm_forward_position_angle_rad = NormalizePiToNegPi((M_PI_2 * 3.0f) + g_arm_wheel_angle_rad);;
	left_wheel_config.wheel_arm_length = g_arm_wheel_circle_radius_mm;
	left_wheel_config.wheel_position = left_wheel_config.servo_position;
	left_wheel_config.wheel_position.x = -(left_wheel_config.wheel_position.x);


	SteeringConfiguration center_wheel_config;
	center_wheel_config.servo_arm_forward_position_angle_rad = g_wheel_arm_forward_angle_position_rad;
	center_wheel_config.servo_arm_length = g_arm_wheel_circle_radius_mm;
	center_wheel_config.servo_position = g_arm_wheel_position;
	center_wheel_config.wheel_arm_forward_position_angle_rad = NormalizePiToNegPi((M_PI_2 * 3.0f));
	center_wheel_config.wheel_arm_length = g_arm_wheel_circle_radius_mm;
	center_wheel_config.wheel_position = g_servo_position;
	center_wheel_config.wheel_position.y = right_wheel_config.wheel_position.y;

	center_wheel_config.wheel_arm_length = fabs(center_wheel_config.wheel_arm_length * sinf(center_wheel_config.wheel_arm_forward_position_angle_rad));



	//result_1 = degrees(WheelAngleToServoRawAngle_rad(radians(-36), right_wheel_config)); // -0.936093390 rad
	//result_2 = degrees(ServoRawAngleToWheelAngle_rad(radians(-36), right_wheel_config)); // 40
	//result_1 = degrees(WheelAngleToServoRawAngle_rad(radians(47), right_wheel_config)); // -0.936093390 rad
	//result_2 = degrees(ServoRawAngleToWheelAngle_rad(radians(47), right_wheel_config)); // -40

	
	//for (int i = -46; i <= 46; i++)
	for (int i = -39; i <= 39; i++)
	//for (int i = (-36 - 4); i <= (48 - 4); i++)
	{

		//right_wheel = degrees(RightWheelAngle(radians(i), WHEEL_BASE_M, DISTANCE_BETWEEN_WHEELS_M));
		servo_2 = i;
		//left_wheel = degrees(ServoRawAngleToWheelAngle_rad(radians(right_wheel), left_wheel_config));
		right_wheel = degrees(ServoRawAngleToWheelAngle_rad(radians(servo_2), right_wheel_config));
		steering_angle = degrees(ServoRawAngleToWheelAngle_rad(radians(right_wheel), center_wheel_config));

		//servo_2 = degrees(WheelAngleToServoRawAngle_rad(radians(right_wheel), right_wheel_config));

		servo_2 = degrees(SteeringAngleToServoAngle_rad(radians(steering_angle), right_wheel_config));
		//servo_2 = degrees(ServoAngleToSteeringAngle_rad(radians(servo_2), right_wheel_config));

		left_wheel = degrees(ServoRawAngleToWheelAngle_rad(radians(right_wheel), left_wheel_config));
		//steering_angle = degrees(RightWheelAngleToVehicleSteeringAngle(radians(right_wheel), WHEEL_BASE_M, DISTANCE_BETWEEN_WHEELS_M));
		left_wheel_angle_from_steering = degrees(LeftWheelAngle(radians(steering_angle), WHEEL_BASE_M, DISTANCE_BETWEEN_WHEELS_M));
		right_wheel_angle_from_steering = degrees(RightWheelAngle(radians(steering_angle), WHEEL_BASE_M, DISTANCE_BETWEEN_WHEELS_M));
		trackwidth = RearWheelTurnRadius(WHEEL_BASE_M, radians(left_wheel)) - RearWheelTurnRadius(WHEEL_BASE_M, radians(right_wheel));
		printf("servo [%d] = wheel_FL: %f\t wheel_FR: %f\t servo: %f\t steering: %f\n", i, left_wheel, right_wheel, servo_2, steering_angle);
		printf("Achermann:\tFL: %f\t\tFR: %f\n\n", left_wheel_angle_from_steering, right_wheel_angle_from_steering);
		//printf("%f\n", trackwidth);
	}
	return 0;
}


#define STEERING_SERVO_ANGLE_MIDDLE     90
#define STEERING_SERVO_ANGLE_MAX_RIGHT  126   // +36 -> -36 going right 126
#define STEERING_SERVO_ANGLE_MAX_LEFT   48     // -47 -> +47 going left 43 //49
#define STEERING_SERVO_ERROR -4.0f

#define STEERING_SERVO_ANGLE_MAX_RIGHT  117   // -> -27 going right 126
#define STEERING_SERVO_ANGLE_MAX_LEFT   45     // -47 -> +45 going left 43 //49
#define STEERING_SERVO_ERROR -6.3f


int main() {
	float arm_wheel_angle;
	float arm_wheel_length;
	arm_wheel_angle = 19.167f;
	arm_wheel_length = 24.5f;
	//arm_wheel_angle = 16.46f;
	//arm_wheel_length = 25.547f;
	SteeringWheel g_steering_wheel(WHEEL_BASE_M, DISTANCE_BETWEEN_WHEELS_M, STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, arm_wheel_angle, arm_wheel_length);
	FILE* fptr;

	fptr = fopen("out.txt", "w+");
	g_steering_wheel.SetRawAngleOffset(STEERING_SERVO_ERROR);

	float servo_angle, servo_raw_angle, steering_wheel_angle, right_wheel_angle, left_wheel, right_wheel_achermann_angle, left_wheel_achermann_angle;
	float valid_angle;
	float right_wheel_error, left_wheel_error;
	fprintf(fptr, "arm_wheel_angle:\t%.3f\n", arm_wheel_angle);
	fprintf(fptr, "arm_wheel_length:\t%.3f\n", arm_wheel_length);
	fprintf(fptr, "\n");
	fprintf(fptr, "steering_angle\tvalid_steering_angle\tachermann_left_wheel\tachermann_right_wheel\tleft_wheel_angle\tright_wheel_angle\tleft_wheel_error\tright_wheel_error\tservo_angle\tservo_raw_angle");
	fprintf(fptr, "\n");
	for (float i = -40; i <= 40; i+=0.1)
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
		fprintf(fptr, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t", (float)i, valid_angle, left_wheel_achermann_angle, right_wheel_achermann_angle, left_wheel, right_wheel_angle, left_wheel_error, right_wheel_error, servo_angle, servo_raw_angle);
		fprintf(fptr, "\n");
		//printf("req_ang [%d] = steer_wheel: %f\t left_wheel: %f\t right_wheel: %f\t servo: %f\t servo_raw: %f\n", i, steering_wheel_angle, left_wheel, right_wheel_angle, servo_angle, servo_raw_angle);
		//printf("Achermann:\tFL: %f\t\tFR: %f\n\n", left_wheel_achermann_angle, right_wheel_achermann_angle);
	}
	fclose(fptr);

	return 0;
}


int main22222() {

	float servo_angle, servo_raw_angle, steering_wheel_angle, right_wheel_angle, left_wheel_angle, right_wheel_achermann_angle, left_wheel_achermann_angle;
	float right_wheel_error, left_wheel_error;
	float valid_angle;
	float average_left_err, average_right_err;
	float max_left_err, max_right_err;
	int i;
	FILE* fptr;

	fptr = fopen("out.txt", "w");

	float start_steer_ang, stop_steer_ang;
	start_steer_ang = 16.0f;
	stop_steer_ang = 21.0f;

	//for (float arm_len = 24.5f; arm_len <= 24.51f; arm_len += 0.01f)
	for (float arm_len = 24.5f; arm_len <= 24.51f; arm_len += 0.01f)
	{

	

	//for (float arm_ang = 19.1f; arm_ang <= 19.167f; arm_ang += 0.01f)
		for (float arm_ang = 19.167f; arm_ang <= 19.267f; arm_ang += 0.01f)
	{

	fprintf(fptr, "ANGLE: %f\n", arm_ang);
	fprintf(fptr, "arm_len: %f\n", arm_len);
	SteeringWheel g_steering_wheel(WHEEL_BASE_M, DISTANCE_BETWEEN_WHEELS_M, STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, arm_ang, arm_len);
	//SteeringWheel g_steering_wheel(WHEEL_BASE_M, DISTANCE_BETWEEN_WHEELS_M, STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);

	g_steering_wheel.SetRawAngleOffset(-4.0f);
	fprintf(fptr, "Steering: ");
	for (i = -40; i <= 40; i++)
	{
		valid_angle = g_steering_wheel.vaildAngleDeg(i);
		g_steering_wheel.setSteeringWheelAngleDeg(valid_angle);

		steering_wheel_angle = g_steering_wheel.getSteeringWheelAngle();
		right_wheel_angle = g_steering_wheel.getRightWheelAngle_deg();
		left_wheel_angle = g_steering_wheel.getLeftWheelAngle_deg();
		right_wheel_achermann_angle = g_steering_wheel.getRightWheelAchermannAngle_deg();
		left_wheel_achermann_angle = g_steering_wheel.getLeftWheelAchermannAngle_deg();
		right_wheel_error = fabsf(right_wheel_achermann_angle - right_wheel_angle);
		left_wheel_error = fabsf(left_wheel_achermann_angle - left_wheel_angle);



		//fprintf(fptr, "%.2f, ", steering_wheel_angle);
		fprintf(fptr, "%.2f\n", steering_wheel_angle);
	}
	
	fprintf(fptr, "\n");
	fprintf(fptr, "LEFT: ");
	for (i = -40; i <= 40; i++)
	{
		valid_angle = g_steering_wheel.vaildAngleDeg(i);
		g_steering_wheel.setSteeringWheelAngleDeg(valid_angle);

		steering_wheel_angle = g_steering_wheel.getSteeringWheelAngle();
		right_wheel_angle = g_steering_wheel.getRightWheelAngle_deg();
		left_wheel_angle = g_steering_wheel.getLeftWheelAngle_deg();
		right_wheel_achermann_angle = g_steering_wheel.getRightWheelAchermannAngle_deg();
		left_wheel_achermann_angle = g_steering_wheel.getLeftWheelAchermannAngle_deg();
		right_wheel_error = fabsf(right_wheel_achermann_angle - right_wheel_angle);
		left_wheel_error = fabsf(left_wheel_achermann_angle - left_wheel_angle);

		//fprintf(fptr, "%.2f, ", left_wheel_error);
		fprintf(fptr, "%.2f\n", left_wheel_error);
	}
	fprintf(fptr, "\n");
	fprintf(fptr, "RIGHT: ");
	average_left_err = 0.0f;
	average_right_err = 0.0f;
	max_left_err = 0.0f;
	max_right_err = 0.0f;
	for (i = -40; i <= 40; i++)
	{
		valid_angle = g_steering_wheel.vaildAngleDeg(i);
		g_steering_wheel.setSteeringWheelAngleDeg(valid_angle);

		steering_wheel_angle = g_steering_wheel.getSteeringWheelAngle();
		right_wheel_angle = g_steering_wheel.getRightWheelAngle_deg();
		left_wheel_angle = g_steering_wheel.getLeftWheelAngle_deg();
		right_wheel_achermann_angle = g_steering_wheel.getRightWheelAchermannAngle_deg();
		left_wheel_achermann_angle = g_steering_wheel.getLeftWheelAchermannAngle_deg();
		right_wheel_error = fabsf(right_wheel_achermann_angle - right_wheel_angle);
		left_wheel_error = fabsf(left_wheel_achermann_angle - left_wheel_angle);

		average_left_err += left_wheel_error;
		average_right_err += right_wheel_error;
		max_left_err = fmaxf(max_left_err, left_wheel_error);
		max_right_err = fmaxf(max_right_err, right_wheel_error);

		//fprintf(fptr, "%.2f, ", right_wheel_error);
		fprintf(fptr, "%.2f\n", right_wheel_error);
	}
	average_left_err = average_left_err / (float)i;
	average_right_err = average_right_err / (float)i;
	fprintf(fptr, "\nAVERAGE_LEFT: %.2f\n", average_left_err);
	fprintf(fptr, "AVERAGE_RIGHT: %.2f\n", average_right_err);
	fprintf(fptr, "MAX_LEFT: %.2f\n", max_left_err);
	fprintf(fptr, "MAX_RIGHT: %.2f\n", max_right_err);
	fprintf(fptr, "\n");
	fprintf(fptr, "\n");

	}
	fprintf(fptr, "===============================================================================\n");
	}
	fclose(fptr);
	return 0;
}