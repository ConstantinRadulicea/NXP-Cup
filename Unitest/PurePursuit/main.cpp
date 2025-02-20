#include "PurePursuitGeometry.h"

#define LANE_WIDTH_M 0.535f
#define WHEEL_BASE_M 0.175
float g_lane_width_vector_unit = 51.39f;
#define MeterToVectorUnit(m) ((float)(m) * ((float)g_lane_width_vector_unit / (float)LANE_WIDTH_M))

Point2D centerRearAxeCarPosition_vectorUnit = Point2D{39.5, -19.214};
LineABC g_middle_lane_line_pixy_1 = points2lineABC(Point2D{0, 31.609}, Point2D{80, -5.591});
Point2D calculated_waypoint = Point2D{ 47.274f, 9.621f};

Point2D carPosition = Point2D{ 39.5, -19.214};

int main() {
	PurePursuitInfo purePersuitInfo;
	float lookAheadDistance = euclidianDistance(carPosition, calculated_waypoint);
	centerRearAxeCarPosition_vectorUnit.x = carPosition.x;
	centerRearAxeCarPosition_vectorUnit.y = carPosition.y - MeterToVectorUnit(WHEEL_BASE_M);
	purePersuitInfo = purePursuitComputeABC(centerRearAxeCarPosition_vectorUnit, g_middle_lane_line_pixy_1, (float)MeterToVectorUnit(WHEEL_BASE_M), lookAheadDistance);




	return 0;
}