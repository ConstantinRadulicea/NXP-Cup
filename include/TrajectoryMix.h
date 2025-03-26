#ifndef __TRAJECTORYMIX_H__
#define __TRAJECTORYMIX_H__

#include "geometry2D.h"
//#include "Config.h"

// rotate first, then apply axis offset
// offset = raw_measurement - correct_measurement;
// correct_measurement = raw_measurement - offset;
typedef struct LineCalibrationData{
  float angle_offset;
  float x_axis_offset;
  float y_axis_offset;
  Point2D rotation_point;
}LineCalibrationData;

#include "GlobalVariables.h"

LineABC closestLineToCurrentTrajectory(LineABC line1, LineABC line2);

LineCalibrationData lineCalibration(LineABC measured_line);

Vector calibrateVector(Vector vec, LineCalibrationData calibration_data);

LineSegment calibrateLineSegment(LineSegment seg, LineCalibrationData calibration_data);

Vector BirdEye_CalibrateVector(struct BirdEyeCalibrationData calib_data, Vector vec);

LineSegment BirdEye_CalibrateLineSegmentScaledToVector(struct BirdEyeCalibrationData calib_data, LineSegment seg);


#endif