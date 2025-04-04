#ifndef __BIRDEYEVIEW_H__
#define __BIRDEYEVIEW_H__

#include "geometry2D.h"

struct track_widths {
    LineSegment upper_segment;
    LineSegment lower_segment;
};

struct BirdEyeCalibrationData {
    Point2D birdeye_src_matrix[4]; // {bottom_left, top_left, bottom_right, top_right}
    Point2D birdeye_dst_matrix[4]; // {bottom_left, top_left, bottom_right, top_right}
    float src_track_width;
    float birdeye_transform_matrix[3][3];
    int valid;
};

int getPerspectiveTransform(Point2D src[4], Point2D dst[4], float H[3][3]);

Point2D applyPerspectiveTransform(float H[3][3], Point2D p);

struct track_widths getTrackWidths(LineSegment left_segment_, LineSegment right_segment_, float frame_height_);

struct track_widths srcViewToRealView(struct track_widths src_view, float real_track_width_m, float frame_width);

struct BirdEyeCalibrationData CalculateBirdEyeCalibration_TrackWidths(struct track_widths src_track_widths, float frame_width, float frame_height, float real_track_width_m);

struct BirdEyeCalibrationData CalculateBirdEyeCalibration_lines(LineSegment left_segment, LineSegment right_segment, float frame_width, float frame_height, float real_track_width_m);

Point2D BirdEye_CalibratePoint(struct BirdEyeCalibrationData calib_data, Point2D point);

LineSegment BirdEye_CalibrateLineSegment(struct BirdEyeCalibrationData calib_data, LineSegment seg);


#endif // !__BIRDEYEVIEW_H__
