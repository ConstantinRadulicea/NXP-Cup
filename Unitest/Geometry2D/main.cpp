#include "geometry2D/geometry2D.h"
#include "memory.h"



struct TrackLines {
	LineSegment left_line;
	LineSegment right_line;
};

/*	1 week before finals, 8.2 run, hard left in intersection fix (maybe) */
struct TrackLines intersectingCornerLinesFiltering (LineSegment _left_line, LineSegment _right_line, LineABC Horizontal_car_line, LineABC vertical_car_line) {
	struct TrackLines result;
	//Point2D carPosition = { 35.0f, 0.0f };
	//int temp_int, temp_int2;
	IntersectionLines line_segments_intersection;

	//LineABC Horizontal_car_line = xAxisABC();
	//Horizontal_car_line.C = -carPosition.y;

	memset(&result, 0, sizeof(result));
	result.left_line = _left_line;
	result.right_line = _right_line;

	if (isValidLineSegment(_left_line) == 0 || isValidLineSegment(_right_line) == 0) {
		return result;
	}

	if (areLineSegmentsEqual(_left_line, _right_line) != 0) {
		return result;
	}

	LineSegment seg1_to_horizontal_line = getLineSegmentFromStartPointAToLine(_left_line, Horizontal_car_line);
	LineSegment seg2_to_horizontal_line = getLineSegmentFromStartPointAToLine(_right_line, Horizontal_car_line);


	if (isValidLineSegment(seg1_to_horizontal_line) == 0) {
		// do something about it
		return result;
	}

	if (isValidLineSegment(seg2_to_horizontal_line) == 0) {
		// do something about it
		return result;
	}

	line_segments_intersection = lineSegmentIntersection(seg1_to_horizontal_line, seg2_to_horizontal_line);


	if (line_segments_intersection.info != INTERSECTION_INFO_ONE_INTERSECTION) {
		return result;
	}

	// check special case

	IntersectionLines seg1_to_car_vertical;
	IntersectionLines seg2_to_car_vertical;


	seg1_to_car_vertical = intersectionLinesABC(vertical_car_line, lineSegmentToLineABC(seg1_to_horizontal_line));
	seg2_to_car_vertical = intersectionLinesABC(vertical_car_line, lineSegmentToLineABC(seg2_to_horizontal_line));


	if (seg1_to_car_vertical.info == INTERSECTION_INFO_ONE_INTERSECTION) {
		if (isPointOnSegment(seg1_to_horizontal_line, seg1_to_car_vertical.point)!= 0) {
			memset(&(result.left_line), 0, sizeof(LineSegment));
		}
	}

	if (seg2_to_car_vertical.info == INTERSECTION_INFO_ONE_INTERSECTION) {
		if (isPointOnSegment(seg2_to_horizontal_line, seg2_to_car_vertical.point) != 0) {
			memset(&(result.right_line), 0, sizeof(LineSegment));
		}
		
	}

	return result;
}



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


	LineABC line1 = points2lineABC({ 10.0f, 0.0f }, { 10.0001f, 100.0f });
	LineABC line2 = points2lineABC({ 50.0f, 0.0f }, { 50.0f, 100.0f });

	int gg = areParallelABC(line1, line2);

	LineABC acute_line;
	LineABC ottuse_line;

	bisectorsOfTwoLinesABC(line1, line2, &acute_line, &ottuse_line);



	struct TrackLines double_checked_lines;

	Point2D carPosition = { (79.0f / 2.0f), 0.0f };
	LineABC Horizontal_car_line = xAxisABC();
	LineABC Vertical_car_line = yAxisABC();

	Vertical_car_line.C = -carPosition.x;


	LineSegment left_line = { { 30.0f, 20.0f }, { 35.0f, 73.0f } };
	LineSegment right_line = { { 21.0f, 21.0f }, { -17.0f, 38.0f } };


	double_checked_lines = intersectingCornerLinesFiltering(left_line, right_line, Horizontal_car_line, Vertical_car_line);

}

