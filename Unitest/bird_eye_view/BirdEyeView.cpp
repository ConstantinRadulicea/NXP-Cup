#include "BirdEyeView.h"
#include "math.h"
#include <stdint.h>


// Solve 8x8 linear system using Gaussian Elimination
void solveLinearSystem(float A[8][9], float* solution) {
    for (int i = 0; i < 8; i++) {
        // Find max row in column i
        int maxRow = i;
        for (int k = i + 1; k < 8; k++) {
            if (fabs(A[k][i]) > fabs(A[maxRow][i])) {
                maxRow = k;
            }
        }
        // Swap rows
        for (int k = i; k < 9; k++) {
            float tmp = A[maxRow][k];
            A[maxRow][k] = A[i][k];
            A[i][k] = tmp;
        }

        // Normalize row
        float divisor = A[i][i];
        for (int k = i; k < 9; k++) {
            A[i][k] /= divisor;
        }

        // Eliminate column
        for (int j = 0; j < 8; j++) {
            if (j != i) {
                float factor = A[j][i];
                for (int k = i; k < 9; k++) {
                    A[j][k] -= factor * A[i][k];
                }
            }
        }
    }

    // Extract solution
    for (int i = 0; i < 8; i++) {
        solution[i] = A[i][8];
    }
}

// Compute Perspective Transformation Matrix
int getPerspectiveTransform(Point2D src[4], Point2D dst[4], float H[3][3]) {
    float A[8][9] = { 0.0f };
    float h[8];

    for (int i = 0; i < 4; i++) {
        float x = src[i].x, y = src[i].y;
        float x_p = dst[i].x, y_p = dst[i].y;

        A[i * 2][0] = x;
        A[i * 2][1] = y;
        A[i * 2][2] = 1.0f;
        A[i * 2][6] = -x * x_p;
        A[i * 2][7] = -y * x_p;
        A[i * 2][8] = x_p;

        A[i * 2 + 1][3] = x;
        A[i * 2 + 1][4] = y;
        A[i * 2 + 1][5] = 1.0f;
        A[i * 2 + 1][6] = -x * y_p;
        A[i * 2 + 1][7] = -y * y_p;
        A[i * 2 + 1][8] = y_p;
    }

    //solveLinearSystem(A, h);
    int res = gaussianElimination8(A, h);

    // Construct Homography Matrix
    H[0][0] = h[0]; H[0][1] = h[1]; H[0][2] = h[2];
    H[1][0] = h[3]; H[1][1] = h[4]; H[1][2] = h[5];
    H[2][0] = h[6]; H[2][1] = h[7]; H[2][2] = 1.0f;
    return res;
}


// Apply Perspective Transformation
Point2D applyPerspectiveTransform(float H[3][3], Point2D p) {
    Point2D result = {};
    float w = H[2][0] * p.x + H[2][1] * p.y + H[2][2];

    if (floatCmp(w, 0.0f) == 0) {
        return result;
    }
    result.x = (H[0][0] * p.x + H[0][1] * p.y + H[0][2]) / w;
    result.y = (H[1][0] * p.x + H[1][1] * p.y + H[1][2]) / w;


    return result;
}

/*

#define FIXED_POINT_SCALE 10000 // scaling factor for 3 decimal places

// Fixed-point version of the Perspective Transformation
Point2D applyPerspectiveTransform(float H[3][3], Point2D p) {
    Point2D result = {};  // Default initialization

    // Convert the point and matrix to fixed-point values
    int32_t p_x = (int32_t)(p.x * FIXED_POINT_SCALE);
    int32_t p_y = (int32_t)(p.y * FIXED_POINT_SCALE);

    // Calculate the value of w using fixed-point
    int32_t w = H[2][0] * p_x + H[2][1] * p_y + (int32_t)(H[2][2] * FIXED_POINT_SCALE);

    // Avoid division by zero
    if (abs(w) < 1) {
        return result;  // Return zeroed result if division by zero would occur
    }

    // Compute the perspective transformation for x and y
    int32_t x_w = H[0][0] * p_x + H[0][1] * p_y + (int32_t)(H[0][2] * FIXED_POINT_SCALE);
    int32_t y_w = H[1][0] * p_x + H[1][1] * p_y + (int32_t)(H[1][2] * FIXED_POINT_SCALE);

    // Perform the division and convert back to float
    result.x = (float)((float)x_w / (float)w);
    result.y = (float)((float)y_w / (float)w);

    return result;
}

*/




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
    if (temp_line_intersection.info == INTERSECTION_INFO_LINES_ARE_EQUAL) {
        return result_track_widths;
    }
    lower_intersect_line = perpendicularToLinePassingThroughPointABC(middle_line, temp_line_intersection.point);


    upper_intersect_line = xAxisABC();
    upper_intersect_line.C = -frame_height_;
    temp_line_intersection = intersectionLinesABC(middle_line, upper_intersect_line);
    if (temp_line_intersection.info == INTERSECTION_INFO_LINES_ARE_EQUAL) {
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
struct track_widths srcViewToRealView(struct track_widths src_view, float real_track_width_m, float frame_width) {
    struct track_widths real_view;
    Point2D upper_midpoint, lower_midpoint, upper_real_modpoint;
    float reference_distance, actual_size, apparent_size, actual_distance;
    LineABC middle_line;
    LineABC temp_line;
    IntersectionPoints2D_2 temp_circle_intersection;
    float lower_segment_length;

    memset(&real_view, 0, sizeof(real_view));


    lower_segment_length = lengthLineSegment(src_view.lower_segment);
    if (floatCmp(lower_segment_length, 0.0f) <= 0) {
        return real_view;
    }

    if (floatCmp(real_track_width_m, 0.0f) <= 0) {
        return real_view;
    }


    upper_midpoint = midPointLineSegment(src_view.upper_segment);
    lower_midpoint = midPointLineSegment(src_view.lower_segment);

    reference_distance = euclidianDistance(upper_midpoint, lower_midpoint);
    actual_size = real_track_width_m;
    apparent_size = lengthLineSegment(src_view.upper_segment);



    if (floatCmp(apparent_size, 0.0f) <= 0) {
        return real_view;
    }
    actual_distance = (actual_size * reference_distance) / apparent_size;

    if (floatCmp(actual_distance, 0.0f) <= 0) {
        return real_view;
    }



    middle_line = points2lineABC(lower_midpoint, upper_midpoint);

    // calculate lower segment
    temp_line = perpendicularToLinePassingThroughPointABC(middle_line, lower_midpoint);
    temp_circle_intersection = intersectionLineCircleABC(lower_midpoint, (actual_size / 2.0f), temp_line);

    if (floatCmp(euclidianDistance(src_view.lower_segment.A, temp_circle_intersection.point1), euclidianDistance(src_view.lower_segment.A, temp_circle_intersection.point2)) < 0)
    {
        real_view.lower_segment.A = temp_circle_intersection.point1;
        real_view.lower_segment.B = temp_circle_intersection.point2;
    }
    else
    {
        real_view.lower_segment.A = temp_circle_intersection.point2;
        real_view.lower_segment.B = temp_circle_intersection.point1;
    }


    // calculate midpoint upper segment
    temp_circle_intersection = intersectionLineCircleABC(lower_midpoint, actual_distance, middle_line);

    if (floatCmp(euclidianDistance(upper_midpoint, temp_circle_intersection.point1), euclidianDistance(upper_midpoint, temp_circle_intersection.point2)) < 0)
    {
        upper_real_modpoint = temp_circle_intersection.point1;
    }
    else
    {
        upper_real_modpoint = temp_circle_intersection.point2;
    }


    // calculate upper segment
    temp_line = perpendicularToLinePassingThroughPointABC(middle_line, upper_real_modpoint);
    temp_circle_intersection = intersectionLineCircleABC(upper_real_modpoint, (actual_size / 2.0f), temp_line);

    if (floatCmp(euclidianDistance(src_view.upper_segment.A, temp_circle_intersection.point1), euclidianDistance(src_view.upper_segment.A, temp_circle_intersection.point2)) < 0)
    {
        real_view.upper_segment.A = temp_circle_intersection.point1;
        real_view.upper_segment.B = temp_circle_intersection.point2;
    }
    else
    {
        real_view.upper_segment.A = temp_circle_intersection.point2;
        real_view.upper_segment.B = temp_circle_intersection.point1;
    }



    real_view.lower_segment.A.x -= lower_midpoint.x;
    real_view.lower_segment.B.x -= lower_midpoint.x;
    real_view.upper_segment.A.x -= lower_midpoint.x;
    real_view.upper_segment.B.x -= lower_midpoint.x;

    float temp_gg = ((float)(lower_midpoint.x) * ((float)actual_size / (float)lower_segment_length));

    real_view.lower_segment.A.x += temp_gg;
    real_view.lower_segment.B.x += temp_gg;
    real_view.upper_segment.A.x += temp_gg;
    real_view.upper_segment.B.x += temp_gg;


    //real_view.lower_segment.A.x = ((float)(real_view.lower_segment.A.x) * ((float)actual_size / (float)lower_segment_length));
    //real_view.lower_segment.B.x = ((float)(real_view.lower_segment.B.x) * ((float)actual_size / (float)lower_segment_length));
    //real_view.upper_segment.A.x = ((float)(real_view.upper_segment.A.x) * ((float)actual_size / (float)lower_segment_length));
    //real_view.upper_segment.B.x = ((float)(real_view.upper_segment.B.x) * ((float)actual_size / (float)lower_segment_length));

    return real_view;
}


struct track_widths srcViewToUncalibratedSrcView(struct track_widths src_view, float frame_height_, float frame_width) {
    struct track_widths uncalibrated_view;
    Point2D upper_midpoint, lower_midpoint;
    float distance;
    float upper_segment_length, lower_segment_length;

    upper_midpoint = midPointLineSegment(src_view.upper_segment);
    lower_midpoint = midPointLineSegment(src_view.lower_segment);

    distance = euclidianDistance(upper_midpoint, lower_midpoint);
    upper_segment_length = lengthLineSegment(src_view.upper_segment);
    lower_segment_length = lengthLineSegment(src_view.lower_segment);





    return uncalibrated_view;
}



struct track_widths realViewToUncalibratedRealView(struct track_widths real_view, float real_track_width_m, float frame_height_, float frame_width) {
    struct track_widths uncalibrated_real_view;





    return uncalibrated_real_view;
}


struct BirdEyeCalibrationData CalculateBirdEyeCalibration_TrackWidths(struct track_widths src_track_widths, float frame_width, float frame_height, float real_track_width_m) {
    struct track_widths dst_track_widths;
    struct BirdEyeCalibrationData data;
    data.valid = 0;

    if (floatCmp(frame_height, 0.0f) <= 0) {
        return data;
    }
    if (floatCmp(real_track_width_m, 0.0f) <= 0) {
        return data;
    }
    if (floatCmp(frame_width, 0.0f) <= 0) {
        return data;
    }

    data.src_track_width = lengthLineSegment(src_track_widths.lower_segment);

    dst_track_widths = srcViewToRealView(src_track_widths, real_track_width_m, frame_width);

    data.birdeye_src_matrix[0] = src_track_widths.lower_segment.A;
    data.birdeye_src_matrix[1] = src_track_widths.upper_segment.A;
    data.birdeye_src_matrix[2] = src_track_widths.lower_segment.B;
    data.birdeye_src_matrix[3] = src_track_widths.upper_segment.B;

    data.birdeye_dst_matrix[0] = dst_track_widths.lower_segment.A;
    data.birdeye_dst_matrix[1] = dst_track_widths.upper_segment.A;
    data.birdeye_dst_matrix[2] = dst_track_widths.lower_segment.B;
    data.birdeye_dst_matrix[3] = dst_track_widths.upper_segment.B;

    data.valid = 0;
    int res = getPerspectiveTransform(data.birdeye_src_matrix, data.birdeye_dst_matrix, data.birdeye_transform_matrix);
    if (res == CONSISTENT_ECUATION_SYSTEM) {
        data.valid = 1;
    }

    return data;
}


struct BirdEyeCalibrationData CalculateBirdEyeCalibration_lines(LineSegment left_segment, LineSegment right_segment, float frame_width, float frame_height, float real_track_width_m) {
    struct track_widths src_track_widths;
    struct BirdEyeCalibrationData data;
    data.valid = 0;

    if (!isValidLineSegment(left_segment)) {
        return data;
    }
    if (!isValidLineSegment(right_segment)) {
        return data;
    }
    if (floatCmp(frame_height, 0.0f) <= 0) {
        return data;
    }

    src_track_widths = getTrackWidths(left_segment, right_segment, frame_height);
    data = CalculateBirdEyeCalibration_TrackWidths(src_track_widths, frame_width, frame_height, real_track_width_m);
    return data;
}


Point2D BirdEye_CalibratePoint(struct BirdEyeCalibrationData calib_data, Point2D point) {
    Point2D result;
    result = point;
    if (calib_data.valid) {
        result = applyPerspectiveTransform(calib_data.birdeye_transform_matrix, point);
    }

    return result;
}


LineSegment BirdEye_CalibrateLineSegment(struct BirdEyeCalibrationData calib_data, LineSegment seg) {
    LineSegment result;
    result = seg;
    if (calib_data.valid) {
        result.A = applyPerspectiveTransform(calib_data.birdeye_transform_matrix, seg.A);
        result.B = applyPerspectiveTransform(calib_data.birdeye_transform_matrix, seg.B);
    }

    return result;
}

