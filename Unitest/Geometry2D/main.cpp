#include "geometry2D/geometry2D.h"





void main() {
	LineSegment main_segment{ {0.0f, 59.0f}, {58.0f, 0.0f} };
	LineABC line;
	Point2D view_point = {39.5f, -19.214f};

	LineSegment result_from_perspective;

	line = points2lineABC({ 80, 0 }, { 0, 80 });

	result_from_perspective = projectSegmentOntoLineFromViewpoint(main_segment, line, view_point);


	int test_gg;
	test_gg = isPointOnSegment(result_from_perspective, { 0.0, 80.0 });
	test_gg = isPointOnLineABC(result_from_perspective.A, line);

	LineSegment temp_seg1;

	temp_seg1 = getLongestReachableSegment(view_point, { {0.0f, 59.0f}, {58.0f, 0.0f} }, { { 80, 0 }, { 0, 80 } });
	temp_seg1 = getLongestReachableSegment(view_point, { {0.0f, 59.0f}, {58.0f, 0.0f} }, { { 80, 0 }, { 80, 0 } });


}

