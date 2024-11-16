#ifndef _STEERINGWHEEL_H_
#define _STEERINGWHEEL_H_
#include "SteeringServo.h"
#include "geometry2D.h"
#include "PurePursuitGeometry.h"


struct SteeringConfiguration {
	Point2D servo_position;
	Point2D wheel_position;
	float servo_arm_length;
	float wheel_arm_length;
	float wheel_arm_forward_position_angle_rad;
	float servo_arm_forward_position_angle_rad;
};

// positive angle: going left
// negative angle: goinf right
static Point2D wheelArmServoRodPosition(float steering_angle, SteeringConfiguration steering_config) {
	steering_angle = NormalizePiToNegPi(steering_angle);
	Point2D servo_position = steering_config.servo_position;
	Point2D arm_wheel_position = steering_config.wheel_position;

	float servo_arm_circle_radius_mm = steering_config.servo_arm_length;
	float arm_wheel_circle_radius_mm = steering_config.wheel_arm_length;

	float servo_arm_to_arm_wheel_rod_length_mm; //= g_servo_arm_to_arm_wheel_rod_length_mm;
	float arm_wheel_angle_position_rad = steering_config.wheel_arm_forward_position_angle_rad;
	float servo_rod_forward_angle_position_rad = steering_config.servo_arm_forward_position_angle_rad;
	float servo_rod_angle_position_rad = steering_config.servo_arm_forward_position_angle_rad;
	float raw_angle = 0.0f;
	Point2D servo_rod_position;
	Point2D arm_wheel_rod_position;
	IntersectionPoints2D_2 temp_intersectionPoints;
	float temp_distance_1, temp_distance_2;
	float cmp_result_1;

	servo_rod_position = circleAngleToPoint2D(servo_position, servo_arm_circle_radius_mm, servo_rod_angle_position_rad);
	arm_wheel_rod_position = circleAngleToPoint2D(arm_wheel_position, arm_wheel_circle_radius_mm, arm_wheel_angle_position_rad);
	servo_arm_to_arm_wheel_rod_length_mm = euclidianDistance(servo_rod_position, arm_wheel_rod_position);


	// the function main logic starts here
	arm_wheel_angle_position_rad = NormalizePiToNegPi(arm_wheel_angle_position_rad + steering_angle);
	arm_wheel_rod_position = circleAngleToPoint2D(arm_wheel_position, arm_wheel_circle_radius_mm, arm_wheel_angle_position_rad);

	temp_intersectionPoints = intersectionBwCircles(servo_position, servo_arm_circle_radius_mm, arm_wheel_rod_position, servo_arm_to_arm_wheel_rod_length_mm);

	temp_distance_1 = euclidianDistance(servo_rod_position, temp_intersectionPoints.point1);
	temp_distance_2 = euclidianDistance(servo_rod_position, temp_intersectionPoints.point2);

	if (temp_distance_1 < temp_distance_2) {
		servo_rod_position = temp_intersectionPoints.point1;
	}
	else {
		servo_rod_position = temp_intersectionPoints.point2;
	}
	return servo_rod_position;
}


// positive angle: going left
// negative angle: goinf right
static float ServoRawAngleToWheelAngle_rad(float raw_angle, SteeringConfiguration steering_config) {
	raw_angle = NormalizePiToNegPi(raw_angle);

	float arm_wheel_forward_angle_position_rad = steering_config.wheel_arm_forward_position_angle_rad;

	Point2D servo_position = steering_config.servo_position;
	Point2D arm_wheel_position = steering_config.wheel_position;

	float servo_arm_circle_radius_mm = steering_config.servo_arm_length;
	float arm_wheel_circle_radius_mm = steering_config.wheel_arm_length;

	float servo_arm_to_arm_wheel_rod_length_mm; //= g_servo_arm_to_arm_wheel_rod_length_mm;
	float arm_wheel_angle_position_rad = steering_config.wheel_arm_forward_position_angle_rad;
	float servo_rod_forward_angle_position_rad = steering_config.servo_arm_forward_position_angle_rad;
	float servo_rod_angle_position_rad = steering_config.servo_arm_forward_position_angle_rad;

	float steering_angle = 0.0f;
	Point2D servo_rod_position;
	Point2D arm_wheel_rod_position;
	IntersectionPoints2D_2 temp_intersectionPoints;
	float temp_distance_1, temp_distance_2;
	float cmp_result_1;

	servo_rod_position = circleAngleToPoint2D(servo_position, servo_arm_circle_radius_mm, servo_rod_angle_position_rad);
	arm_wheel_rod_position = circleAngleToPoint2D(arm_wheel_position, arm_wheel_circle_radius_mm, arm_wheel_angle_position_rad);
	servo_arm_to_arm_wheel_rod_length_mm = euclidianDistance(servo_rod_position, arm_wheel_rod_position);

	// the function main logic starts here
	servo_rod_angle_position_rad = NormalizePiToNegPi(servo_rod_angle_position_rad + raw_angle);
	servo_rod_position = circleAngleToPoint2D(servo_position, servo_arm_circle_radius_mm, servo_rod_angle_position_rad);

	temp_intersectionPoints = intersectionBwCircles(arm_wheel_position, arm_wheel_circle_radius_mm, servo_rod_position, servo_arm_to_arm_wheel_rod_length_mm);

	temp_distance_1 = euclidianDistance(arm_wheel_rod_position, temp_intersectionPoints.point1);
	temp_distance_2 = euclidianDistance(arm_wheel_rod_position, temp_intersectionPoints.point2);

	if (temp_distance_1 < temp_distance_2) {
		arm_wheel_rod_position = temp_intersectionPoints.point1;
	}
	else {
		arm_wheel_rod_position = temp_intersectionPoints.point2;
	}

	arm_wheel_angle_position_rad = circlePoint2DToAngle(arm_wheel_position, arm_wheel_rod_position);
	steering_angle = NormalizePiToNegPi(arm_wheel_angle_position_rad - arm_wheel_forward_angle_position_rad);
	return steering_angle;
}



// positive angle: going left
// negative angle: goinf right
static float WheelAngleToServoRawAngle_rad(float steering_angle, SteeringConfiguration steering_config) {
	steering_angle = NormalizePiToNegPi(steering_angle);
	Point2D servo_position = steering_config.servo_position;


	float servo_rod_forward_angle_position_rad = steering_config.servo_arm_forward_position_angle_rad;
	float servo_rod_angle_position_rad;
	float raw_angle = 0.0f;
	Point2D servo_rod_position;


	servo_rod_position = wheelArmServoRodPosition(steering_angle, steering_config);
	servo_rod_angle_position_rad = circlePoint2DToAngle(servo_position, servo_rod_position);

	raw_angle = NormalizePiToNegPi(servo_rod_angle_position_rad - servo_rod_forward_angle_position_rad);

	return raw_angle;
}



class SteeringWheel
{
public:
	SteeringServo steering_servo;
	SteeringWheel(float wheel_base, float track_width, unsigned int servo_max_left_raw_angle = 0, unsigned int servo_middle_raw_angle = 90, unsigned int servo_max_right_raw_angle = 180)
		: steering_servo(servo_max_left_raw_angle, servo_middle_raw_angle, servo_max_right_raw_angle)
	{
		// positive angle: going left
		// negative angle: goinf right
		// servo angle: [60.33; -48.962376], wheel angle: [41.116337; -65.044365]
		Point2D g_servo_position = Point2D{ 0.0f, 0.0f };
		Point2D g_arm_wheel_position = Point2D{ 52.392, -6.0f };
		float g_servo_arm_circle_radius_mm = 24.0f;	// 24mm, 20mm
		float g_arm_wheel_circle_radius_mm = 25.547f;
		float g_arm_wheel_angle_rad = NormalizePiToNegPi(radians(16.46f));		//16.46

		// float g_servo_arm_to_arm_wheel_rod_length_mm = 45.6188202f;	// not necessary to calculate manually, already done automatically
		float g_servo_rod_forward_angle_position_rad = NormalizePiToNegPi((M_PI_2 * 3.0f));
		float g_arm_wheel_forward_angle_position_rad = NormalizePiToNegPi((M_PI_2 * 3.0f) - g_arm_wheel_angle_rad);

		this->steering_config.servo_arm_forward_position_angle_rad = g_servo_rod_forward_angle_position_rad;
		this->steering_config.servo_arm_length = g_servo_arm_circle_radius_mm;
		this->steering_config.servo_position = g_servo_position;
		this->steering_config.wheel_arm_forward_position_angle_rad = g_arm_wheel_forward_angle_position_rad;
		this->steering_config.wheel_arm_length = g_arm_wheel_circle_radius_mm;
		this->steering_config.wheel_position = g_arm_wheel_position;

		this->wheel_base = wheel_base;
		this->track_width = track_width;
		this->steering_wheel_angle = 0.0f;

		this->SteeringWheel_MaxLeftAngle = degrees(RightWheelAngleToVehicleSteeringAngle(ServoRawAngleToWheelAngle_rad(radians(this->steering_servo.getMaxLeftAngleDeg()), this->steering_config), this->wheel_base, this->track_width));
		this->SteeringWheel_MaxRightAngle = degrees(RightWheelAngleToVehicleSteeringAngle(ServoRawAngleToWheelAngle_rad(radians(this->steering_servo.getMaxRightAngleDeg()), this->steering_config), this->wheel_base, this->track_width));
	}

	void setWheelBase(float wheelBase) {
		this->wheel_base = wheelBase;
	}
	void setTrackWidth(float trackWidth) {
		this->track_width = trackWidth;
	}
	void setSteeringWheelAngleDeg(float steering_wheel_angle) {
		steering_wheel_angle = this->vaildAngleDeg(steering_wheel_angle);
		float right_wheel_angle_rad = RightWheelAngle(radians(steering_wheel_angle), this->wheel_base, this->track_width);
		float servo_raw_angle_deg = degrees(WheelAngleToServoRawAngle_rad(right_wheel_angle_rad, this->steering_config));
		this->steering_servo.setAngleDeg(servo_raw_angle_deg);
		this->steering_wheel_angle = steering_wheel_angle;
	}

	float getSteeringWheelAngle() {
		return this->steering_wheel_angle;
	}

	float vaildAngleDeg(float angle) {
		float valid_angle;
		float min_steering_angle;
		float max_steering_angle;
		min_steering_angle = MIN(this->SteeringWheel_MaxLeftAngle, this->SteeringWheel_MaxRightAngle);
		max_steering_angle = MAX(this->SteeringWheel_MaxLeftAngle, this->SteeringWheel_MaxRightAngle);

		valid_angle = MAX(MIN(angle, max_steering_angle), min_steering_angle);
		return valid_angle;
	}

	// -1: left, 0: forward, 1: right
	static int AngleToDirectionDeg(float angle) {
		int cmp_result = floatCmp(angle, 0.0);
		if (cmp_result > 0) {
			return -1;
		}
		else if (cmp_result < 0.0) {
			return 1;
		}
		else {
			return 0;
		}
	}

private:
	
	SteeringConfiguration steering_config;
	float wheel_base;
	float track_width;
	float steering_wheel_angle;

	float SteeringWheel_MaxLeftAngle;
	float SteeringWheel_MaxRightAngle;


};

#endif