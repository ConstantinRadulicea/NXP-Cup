#include "BirdEyeView.h"
#include <stdio.h>


/*
 left_line:
    P1(4; 1)
    P2(17; 44)

 right_line:
    P1(63; 1)
    P2(48; 44)


 lower_size: 59
 upper_size: 31

*/

LineSegment left_segment = LineSegment{ Point2D{4.0, 1.0}, Point2D{17.0, 44.0} };
LineSegment right_segment = LineSegment{ Point2D{63.0, 1.0}, Point2D{48.0, 44.0} };


LineABC getMiddleLine(LineSegment segment_1, LineSegment segment_2) {
    LineABC leftLine, rightLine, middleLine_, acuteAngleBisector, ottuseAngleBisector;
    LineSegment leftVector_ = segment_1;
    LineSegment rightVector_ = segment_2;

    memset(&middleLine_, 0, sizeof(LineABC));

    if (!isValidLineSegment(leftVector_) || !isValidLineSegment(rightVector_)) {
        return middleLine_;
    }

    leftLine = lineSegmentToLineABC(leftVector_);
    rightLine = lineSegmentToLineABC(rightVector_);

    bisectorsOfTwoLinesABC(leftLine, rightLine, &acuteAngleBisector, &ottuseAngleBisector);

    if ((floatCmp(ottuseAngleBisector.Ax, 0.0f) == 1) && (floatCmp(ottuseAngleBisector.By, 0.0f) == 1))
    {
        middleLine_ = acuteAngleBisector;
    }
    else if (fabsf(angleBetweenLinesABC(leftLine, acuteAngleBisector)) < fabsf(angleBetweenLinesABC(leftLine, ottuseAngleBisector)))
    {
        middleLine_ = acuteAngleBisector;
    }
    else {
        middleLine_ = ottuseAngleBisector;
    }
    return middleLine_;
}


struct track_widths {
    LineSegment upper_segment;
    LineSegment lower_segment;
};

struct track_widths getTrackWidths(LineSegment left_segment_, LineSegment right_segment_, float frame_height_) {
    LineABC left_line;
    LineABC right_line;
    LineABC middle_line;
    LineABC lower_intersect_line, upper_intersect_line;
    IntersectionLines temp_line_intersection;
    LineSegment upper_segment, lower_segment;

    struct track_widths result_track_widths;


    memset(&result_track_widths, 0, sizeof(result_track_widths));


    if (areLineSegmentsEqual(left_segment_, right_segment_)) {
        return result_track_widths;
    }

    left_line = lineSegmentToLineABC(left_segment_);
    right_line = lineSegmentToLineABC(right_segment_);

    if (areLinesEqual(left_line, right_line)) {
        return result_track_widths;
    }

    middle_line = getMiddleLine(left_segment_, right_segment_);
    if (!isValidLineABC(middle_line)) {
        return result_track_widths;
    }

    lower_intersect_line = xAxisABC();
    temp_line_intersection = intersectionLinesABC(middle_line, lower_intersect_line);
    if (temp_line_intersection.info == 2) {
        return result_track_widths;
    }
    lower_intersect_line = perpendicularToLinePassingThroughPointABC(middle_line, temp_line_intersection.point);


    upper_intersect_line = xAxisABC();
    upper_intersect_line.C = -frame_height_;
    temp_line_intersection = intersectionLinesABC(middle_line, upper_intersect_line);
    if (temp_line_intersection.info == 2) {
        return result_track_widths;
    }
    upper_intersect_line = perpendicularToLinePassingThroughPointABC(middle_line, temp_line_intersection.point);

    
    temp_line_intersection = intersectionLinesABC(left_line, lower_intersect_line);
    lower_segment.A = temp_line_intersection.point;

    temp_line_intersection = intersectionLinesABC(right_line, lower_intersect_line);
    lower_segment.B = temp_line_intersection.point;

    temp_line_intersection = intersectionLinesABC(left_line, upper_intersect_line);
    upper_segment.A = temp_line_intersection.point;

    temp_line_intersection = intersectionLinesABC(right_line, upper_intersect_line);
    upper_segment.B = temp_line_intersection.point;

    result_track_widths.lower_segment = lower_segment;
    result_track_widths.upper_segment = upper_segment;

    return result_track_widths;
}



/*
apparent_size = (actual_size * reference_distance) / actual_distance

acctual_distance = (actual_size * reference_distance) / apparent_size



*/
struct track_widths srcViewToRealView(struct track_widths src_view, float real_track_width_m) {
    struct track_widths real_view;
    Point2D upper_midpoint, lower_midpoint;
    float apparent_distance;

    memset(&real_view, 0, sizeof(real_view));

    upper_midpoint = midPointLineSegment(src_view.upper_segment);
    lower_midpoint = midPointLineSegment(src_view.lower_segment);

    apparent_distance = euclidianDistance(upper_midpoint, lower_midpoint);



    return real_view;
}


int main() {

    Point2D src[4] = { {4, 1}, {17, 44}, {50, 400}, {550, 400} };
    Point2D dst[4] = { {150, 0}, {450, 0}, {150, 400}, {450, 400} };

    float H[3][3];
    getPerspectiveTransform(src, dst, H);

    printf("Perspective Transformation Matrix:\n");
    for (int i = 0; i < 3; i++) {
        printf("[%f %f %f]\n", H[i][0], H[i][1], H[i][2]);
    }

    Point2D testPoint = { 300, 300 };
    Point2D transformed = applyPerspectiveTransform(H, testPoint);
    printf("Transformed (%.2f, %.2f) -> (%.2f, %.2f)\n", testPoint.x, testPoint.y, transformed.x, transformed.y);

	return 0;
}

