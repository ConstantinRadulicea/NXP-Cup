#include "PurePursuitGeometry.h"
#include "math.h"
#include "stdio.h"

#define LANE_WIDTH_M 0.535f
#define WHEEL_BASE_M 0.175
float g_lane_width_vector_unit = 51.39f;
#define MeterToVectorUnit(m) ((float)(m) * ((float)g_lane_width_vector_unit / (float)LANE_WIDTH_M))

//Point2D centerRearAxeCarPosition_vectorUnit = Point2D{39.5, -19.214};
//LineABC g_middle_lane_line_pixy_1 = points2lineABC(Point2D{0, 31.609}, Point2D{80, -5.591});
//Point2D calculated_waypoint = Point2D{ 47.274f, 9.621f};
//Point2D carPosition = Point2D{ 39.5, -19.214};

/*
0.465x+1y-31.6089993=0
(x-(39.5))^{2}+(y-(-19.2139))^{2}=(29.8645668)^{2}

*/



Point2D centerRearAxeCarPosition_vectorUnit = Point2D{ 39.5, 0};
LineABC g_middle_lane_line_pixy_1 = points2lineABC(Point2D{ -50, 316.834 }, Point2D{ 150, -167.966 });
Point2D calculated_waypoint = Point2D{ 60.184f, 49.731f };
Point2D carPosition = Point2D{ 39.5, 0};

/*
2.424x+1y-195.634003=0
(x-(39.5))^{2}+(y-(0))^{2}=(53.8714638)^{2}

*/


int main() {
	PurePursuitInfo purePersuitInfo;
	float lookAheadDistance = euclidianDistance(carPosition, calculated_waypoint);
	lookAheadDistance = 0.0f;
	centerRearAxeCarPosition_vectorUnit.x = carPosition.x;
	centerRearAxeCarPosition_vectorUnit.y = carPosition.y - MeterToVectorUnit(WHEEL_BASE_M);
	purePersuitInfo = purePursuitComputeABC(centerRearAxeCarPosition_vectorUnit, g_middle_lane_line_pixy_1, (float)MeterToVectorUnit(WHEEL_BASE_M), lookAheadDistance);



	float minDistance = 0.1f;
	float maxDistance = 0.4f;
	float distanceSpan = maxDistance - minDistance;


	for (int i = -90; i <= 90; i++)
	{
		float angleCurrentTrajectoryAndMiddleLane = radians(i);

		float newLookAheadDistance = minDistance + ((((float)M_PI_4 - angleCurrentTrajectoryAndMiddleLane) / (float)M_PI_4) * distanceSpan);

		newLookAheadDistance = MAX(newLookAheadDistance, minDistance);
		newLookAheadDistance = MIN(newLookAheadDistance, maxDistance);

		printf("%d -> %f\n", i, newLookAheadDistance);
	}






	return 0;
}