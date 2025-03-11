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
#define TRACK_WIDTH_M (0.137f)	//distance between wheels




#define STEERING_SERVO_ANGLE_MIDDLE     90
#define STEERING_SERVO_ANGLE_MAX_RIGHT  126   // +36 -> -36 going right 126
#define STEERING_SERVO_ANGLE_MAX_LEFT   48     // -47 -> +47 going left 43 //49
#define STEERING_SERVO_ERROR -4.0f

#define STEERING_SERVO_ANGLE_MAX_RIGHT  140   // -> -27 going right 126
#define STEERING_SERVO_ANGLE_MAX_LEFT   40     // -47 -> +45 going left 43 //49
#define STEERING_SERVO_ERROR -10.0f



int main() {
	Point2D g_servo_position = Point2D{ 0.0f, -(23.142f + 6.4f) };
	Point2D g_arm_wheel_position = Point2D{ (52.0f - 8.044f), ( - 6.4f - 23.142 )};

	LineABC temo_line1 = points2lineABC(g_servo_position, g_arm_wheel_position);
	float ffff = angleBetweenLinesABC(xAxisABC(), temo_line1);
	ffff = euclidianDistance(g_servo_position, g_arm_wheel_position);
	g_servo_position = Point2D{ 0.0f, -(24.0f) };
	ffff = euclidianDistance(g_servo_position, g_arm_wheel_position);

	float arm_wheel_angle;
	float arm_wheel_length;
	arm_wheel_angle = 19.167f;
	arm_wheel_length = 24.5f;
	//arm_wheel_angle = 16.46f;
	//arm_wheel_length = 25.547f;
	SteeringWheel g_steering_wheel(WHEEL_BASE_M, TRACK_WIDTH_M, STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, arm_wheel_angle, arm_wheel_length);
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
		//g_steering_wheel.SetRawAngleOffset(0.0f);
		//g_steering_wheel.setMaxLeftAngle_deg(40.0f);
		//g_steering_wheel.setMaxRightAngle_deg(-40.0f);

		valid_angle = g_steering_wheel.vaildAngleDeg(i);
		//valid_angle = i;
		printf("%f\n", valid_angle);
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

