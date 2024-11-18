
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
		printf("%f\t%f\n", left_wheel_angle_from_steering, right_wheel_angle_from_steering);
		printf("%f\n", trackwidth);
	}
	return 0;
}


#define STEERING_SERVO_ANGLE_MIDDLE     90		// good angle 86
#define STEERING_SERVO_ANGLE_MAX_RIGHT  125   // +36 -> -36 going right 126
#define STEERING_SERVO_ANGLE_MAX_LEFT   48     // -47 -> +47 going left 43 //48

int main() {
	SteeringWheel g_steering_wheel(WHEEL_BASE_M, DISTANCE_BETWEEN_WHEELS_M, STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);
	//g_steering_wheel.setTrackWidth(DISTANCE_BETWEEN_WHEELS_M);
	//g_steering_wheel.setWheelBase(WHEEL_BASE_M);
	
	g_steering_wheel.SetRawAngleOffset(-4.0f);

	float servo_angle, servo_raw_angle, steering_wheel_angle, right_wheel_angle, left_wheel;

	SteeringConfiguration left_wheel_config;
	left_wheel_config.servo_arm_forward_position_angle_rad = g_wheel_arm_forward_angle_position_rad;
	left_wheel_config.servo_arm_length = g_arm_wheel_circle_radius_mm;
	left_wheel_config.servo_position = g_arm_wheel_position;
	left_wheel_config.wheel_arm_forward_position_angle_rad = NormalizePiToNegPi((M_PI_2 * 3.0f) + g_arm_wheel_angle_rad);
	left_wheel_config.wheel_arm_length = g_arm_wheel_circle_radius_mm;
	left_wheel_config.wheel_position = left_wheel_config.servo_position;
	left_wheel_config.wheel_position.x = -(left_wheel_config.wheel_position.x);
	//float g_steering_angle_rad = radians(g_steering_wheel.vaildAngleDeg(degrees(0.28077)));
	float valid_angle;
	for (int i = -60; i <= 60; i++)
	{
		g_steering_wheel.SetRawAngleOffset(-4.0f);
		valid_angle = g_steering_wheel.vaildAngleDeg(i);
		g_steering_wheel.setSteeringWheelAngleDeg(valid_angle);

		servo_angle = g_steering_wheel.steering_servo.getAngleDeg();
		servo_raw_angle = g_steering_wheel.steering_servo.getRawAngleDeg();
		steering_wheel_angle = g_steering_wheel.getSteeringWheelAngle();
		right_wheel_angle = 0;
		left_wheel = degrees(ServoRawAngleToWheelAngle_rad(radians(right_wheel_angle), left_wheel_config));
		printf("req_ang [%d] = steer_wheel: %f\t left_wheel: %f\t right_wheel: %f\t servo: %f\t servo_raw: %f\n", i, steering_wheel_angle, left_wheel, right_wheel_angle, servo_angle, servo_raw_angle);
	}
	return 0;
}