/*
* Copyright 2023 Constantin Dumitru Petre RĂDULICEA
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*   http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

// https://taketake2.com/S19_en.html



#ifndef __PUREPURSUITGEOMETRY_H__
#define __PUREPURSUITGEOMETRY_H__

#include "geometry2D.h"

typedef struct PurePursuitInfo {
	Point2D frontAxePosition;
	Point2D rearAxePosition;
	Point2D nextWayPoint;
	Point2D turnPoint;
	float distanceToWayPoint;
	float lookAheadDistance;
	float TrajectoryToWayPointAngle;
	float steeringAngle;
	float wheelBase;
	float rearWheelTurnRadius;
	float manouvreLength;
}PurePursuitInfo;

static float carTrajectoryAndWayPointAngle(Point2D carPos, Point2D nextWayPoint) {
	float lookAheadDistance, TrajectoryToWayPointAngle;
	Point2D temp;

	lookAheadDistance = euclidianDistance(carPos, nextWayPoint);
	temp = carPos;
	temp.y += lookAheadDistance;
	TrajectoryToWayPointAngle = triangleAngleA(lookAheadDistance, euclidianDistance(nextWayPoint, temp), lookAheadDistance);

	if (floatCmp(carPos.x, nextWayPoint.x) < 0) {
		TrajectoryToWayPointAngle = -TrajectoryToWayPointAngle;
	}
	return TrajectoryToWayPointAngle;
}

static float steeringWheelAngle(float TrajectoryToWayPointAngle, float wheelBase, float nextWayPointDistance) {
	float angle;
	float temp_float;
	if (floatCmp(nextWayPointDistance, 0.0f) == 0) {
		return 0.0f;
	}

	angle = atanf((2.0f * wheelBase * sinf(TrajectoryToWayPointAngle)) / nextWayPointDistance);
	return angle;
}


static float turnRadiusByWaypoint(float TrajectoryToWayPointAngle, float wheelBase, float nextWayPointDistance) {
	float angle;
	float temp_float;

	temp_float = sinf(TrajectoryToWayPointAngle);
	if (floatCmp(temp_float, 0.0f) == 0) {
		return 0.0f;
	}
	angle = (nextWayPointDistance / (2.0f * sinf(TrajectoryToWayPointAngle)));
	return angle;
}

static float FrontWheelTurnRadius(float wheelBase, float turnAngle) {
	float angle;
	float temp_sin = sinf(turnAngle);
	if (floatCmp(turnAngle, 0.0f) == 0) {
		return -1.0f;
	}
	if (floatCmp(temp_sin, 0.0f) == 0) {
		return 0.0f;
	}

	angle = (wheelBase / temp_sin);

	angle = fabsf(angle);
	return angle;
}

static float RearWheelTurnRadius(float wheelBase, float turnAngle) {
	float angle;
	//float temp_sin = sinf(turnAngle);
	if (floatCmp(turnAngle, 0.0f) == 0) {
		return -1.0f;
	}
	float temp_sin = tanf(turnAngle);
	if (floatCmp(temp_sin, 0.0f) == 0) {
		return 0.0f;
	}

	//angle = (wheelBase / tanf(turnAngle));
	angle = (wheelBase / temp_sin);

	angle = fabsf(angle);
	return angle;
}

static float RearWheelTurnRadiusToFrontWheelTurnRadius(float wheelBase, float rear_wheel_turn_radius) {
	if (floatCmp(rear_wheel_turn_radius, 0.0f) < 0) {
		return -1.0f;
	}
	return sqrtf((wheelBase * wheelBase) + (rear_wheel_turn_radius * rear_wheel_turn_radius));
}

static float TurnRadiusToSteeringAngle(float wheelBase, float turn_radius) {
	float steer_angle;
	if (floatCmp(turn_radius, 0.0f) <= 0) {
		return 0.0f;
	}
	steer_angle = asinf(wheelBase / turn_radius);
	steer_angle = fabsf(steer_angle);
	return steer_angle;
}

static float purePursuitComputeSteeringWheelAngle(Point2D carPos, LineMQ wayPoints, float wheelBase, float lookAheadDistance) {
	float temp;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;

	temp = distance2lineMQ(carPos, wayPoints);

	if (floatCmp(temp, lookAheadDistance) >= 0) {
		lookAheadDistance = temp + (temp * 0.1f);
	}

	intersectionPoints = intersectionLineCircleMQ(carPos, lookAheadDistance, wayPoints);

	if (floatCmp(intersectionPoints.point1.y, intersectionPoints.point2.y) > 0) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}

	temp = carTrajectoryAndWayPointAngle(carPos, nextWayPoint);
	return steeringWheelAngle(temp, wheelBase, lookAheadDistance);
}

static PurePursuitInfo purePursuitComputeMQ(Point2D rearAxePosition, LineMQ wayPoints, float wheelBase, float lookAheadDistance) {
	float temp;
	float distance_bw_rearAxe_and_waipoint;
	PurePursuitInfo info;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;
	Point2D frontAxePosition;
	frontAxePosition = rearAxePosition;
	frontAxePosition.y += wheelBase;

	info.frontAxePosition = frontAxePosition;

	temp = distance2lineMQ(frontAxePosition, wayPoints);
	info.distanceToWayPoint = temp;
	if (floatCmp(temp, lookAheadDistance) >= 0) {
		lookAheadDistance = temp + (temp * 0.25f);
	}

	intersectionPoints = intersectionLineCircleMQ(frontAxePosition, lookAheadDistance, wayPoints);
	if (floatCmp(intersectionPoints.point1.y, intersectionPoints.point2.y) > 0) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}
	info.TrajectoryToWayPointAngle = carTrajectoryAndWayPointAngle(rearAxePosition, nextWayPoint);
	distance_bw_rearAxe_and_waipoint = euclidianDistance(rearAxePosition, nextWayPoint);
	info.steeringAngle = steeringWheelAngle(info.TrajectoryToWayPointAngle, wheelBase, distance_bw_rearAxe_and_waipoint);

	info.rearAxePosition = rearAxePosition;
	info.nextWayPoint = nextWayPoint;
	info.lookAheadDistance = lookAheadDistance;
	info.wheelBase = wheelBase;
	info.rearWheelTurnRadius = RearWheelTurnRadius(wheelBase, info.steeringAngle);
	info.manouvreLength = fabsf(((2.0f * M_PI * info.rearWheelTurnRadius) * info.TrajectoryToWayPointAngle) / (2.0f * M_PI));
	info.turnPoint = rearAxePosition;



	if (floatCmp(info.TrajectoryToWayPointAngle, 0.0f) < 0) {
		info.turnPoint.x += info.rearWheelTurnRadius;
	}
	else {
		info.turnPoint.x -= info.rearWheelTurnRadius;
	}

	return info;
}

static PurePursuitInfo purePursuitComputeABC(Point2D rearAxePosition, LineABC wayPoints, float wheelBase, float _lookAheadDistance) {
	float temp;
	PurePursuitInfo info;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;
	Point2D frontAxePosition;
	float distance_bw_rearAxe_and_waipoint;
	float distance_to_line, min_lookahead_distance;
	frontAxePosition = rearAxePosition;
	frontAxePosition.y += wheelBase;

	info.frontAxePosition = frontAxePosition;

	distance_to_line = distance2lineABC(frontAxePosition, wayPoints);
	min_lookahead_distance = (sqrtf((distance_to_line * distance_to_line) / 2.0f) * 2.0f);

	float lookAheadDistance = MAX(min_lookahead_distance, _lookAheadDistance);

	intersectionPoints = intersectionLineCircleABC(frontAxePosition, lookAheadDistance, wayPoints);
	if (floatCmp(intersectionPoints.point1.y, intersectionPoints.point2.y) > 0) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}
	info.TrajectoryToWayPointAngle = carTrajectoryAndWayPointAngle(rearAxePosition, nextWayPoint);
	distance_bw_rearAxe_and_waipoint = euclidianDistance(rearAxePosition, nextWayPoint);
	info.steeringAngle = steeringWheelAngle(info.TrajectoryToWayPointAngle, wheelBase, distance_bw_rearAxe_and_waipoint);

	info.rearAxePosition = rearAxePosition;
	info.nextWayPoint = nextWayPoint;
	info.lookAheadDistance = lookAheadDistance;
	info.wheelBase = wheelBase;
	info.rearWheelTurnRadius = RearWheelTurnRadius(wheelBase, info.steeringAngle);
	info.manouvreLength = fabsf(((2.0f * M_PI * info.rearWheelTurnRadius) * info.TrajectoryToWayPointAngle) / (2.0f * M_PI));
	info.turnPoint = rearAxePosition;
	info.distanceToWayPoint = euclidianDistance(nextWayPoint, frontAxePosition);

	if (floatCmp(info.TrajectoryToWayPointAngle, 0.0f) < 0) {
		info.turnPoint.x += info.rearWheelTurnRadius;
	}
	else {
		info.turnPoint.x -= info.rearWheelTurnRadius;
	}

	return info;
}


static float carTurnMaxSpeed(float _turn_radius, float _friction_coefficient, float _downward_acceleration) {
	float _max_speed;
	_max_speed = sqrtf(fabs(_friction_coefficient * _turn_radius * _downward_acceleration));
	return _max_speed;
}


// positive angle: going left
// negative angle: going right
static float LeftWheelAngle(float steering_angle, float wheel_base, float track_width) {
	float left_wheel_turn_radius;
	int cmp_result;
	float left_wheel_steering_angle;
	float rear_wheel_turn_radius;

	if (floatCmp(track_width, 0.0f) == 0) {
		return 0;
	}

	if (floatCmp(steering_angle, 0.0f) == 0) {
		return 0;
	}
	rear_wheel_turn_radius = RearWheelTurnRadius(wheel_base, steering_angle);
	if (floatCmp(rear_wheel_turn_radius, 0.0f) < 0) {
		return 0;
	}

	cmp_result = floatCmp(steering_angle, 0.0f);
	if (cmp_result > 0)	// going left
	{
		rear_wheel_turn_radius = rear_wheel_turn_radius - (track_width / 2.0f);
	}
	else if (cmp_result < 0) // going right
	{
		rear_wheel_turn_radius = rear_wheel_turn_radius + (track_width / 2.0f);
	}
	else
	{
		return 0.0f;
	}

	left_wheel_turn_radius = RearWheelTurnRadiusToFrontWheelTurnRadius(wheel_base, rear_wheel_turn_radius);
	left_wheel_steering_angle = TurnRadiusToSteeringAngle(wheel_base, left_wheel_turn_radius);

	if (cmp_result < 0) // going right
	{
		left_wheel_steering_angle = -left_wheel_steering_angle;
	}

	return left_wheel_steering_angle;
}


// positive angle: going left
// negative angle: going right
static float RightWheelAngle(float steering_angle, float wheel_base, float track_width) {
	float left_wheel_turn_radius;
	int cmp_result;
	float left_wheel_steering_angle;
	float rear_wheel_turn_radius;

	if (floatCmp(track_width, 0.0f) == 0) {
		return 0;
	}


	if (floatCmp(steering_angle, 0.0f) == 0) {
		return 0;
	}

	rear_wheel_turn_radius = RearWheelTurnRadius(wheel_base, steering_angle);

	if (floatCmp(rear_wheel_turn_radius, 0.0f) < 0) {
		return 0;
	}

	cmp_result = floatCmp(steering_angle, 0.0f);
	if (cmp_result > 0)	// going left
	{
		rear_wheel_turn_radius = rear_wheel_turn_radius + (track_width / 2.0f);
	}
	else if (cmp_result < 0) // going right
	{
		rear_wheel_turn_radius = rear_wheel_turn_radius - (track_width / 2.0f);
	}
	else
	{
		return 0.0f;
	}

	left_wheel_turn_radius = RearWheelTurnRadiusToFrontWheelTurnRadius(wheel_base, rear_wheel_turn_radius);
	left_wheel_steering_angle = TurnRadiusToSteeringAngle(wheel_base, left_wheel_turn_radius);

	if (cmp_result < 0) // going right
	{
		left_wheel_steering_angle = -left_wheel_steering_angle;
	}

	return left_wheel_steering_angle;
}


// positive angle: going left
// negative angle: going right
static float RightWheelAngleToVehicleSteeringAngle(float right_wheel_angle, float wheel_base, float track_width) {
	return LeftWheelAngle(right_wheel_angle, wheel_base, track_width);
}



#endif // !__PUREPURSUITGEOMETRY_H__
