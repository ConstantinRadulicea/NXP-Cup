#ifndef __TRAJECTORYMIX_H__
#define __TRAJECTORYMIX_H__

#include "geometry2D.h"
#include "GlobalVariables.h"

LineABC closestLineToCurrentTrajectory(LineABC line1, LineABC line2){
  LineABC upper_line, lower_line, horizontal_middle_line, calibration_line;
  IntersectionLines upper_intersection, lower_intersection, left_lane_line_intersection, right_lane_line_intersection;

  horizontal_middle_line = xAxisABC();
  horizontal_middle_line.C = -SCREEN_CENTER_Y;

  left_lane_line_intersection = intersectionLinesABC(line1, horizontal_middle_line);
  right_lane_line_intersection = intersectionLinesABC(line2, horizontal_middle_line);

  if (left_lane_line_intersection.info == 0 && right_lane_line_intersection.info == 0) {
    if (euclidianDistance(left_lane_line_intersection.point, Point2D{SCREEN_CENTER_X, SCREEN_CENTER_Y}) < euclidianDistance(right_lane_line_intersection.point, Point2D{SCREEN_CENTER_X, SCREEN_CENTER_Y})) {
      calibration_line = line1;
    }
    else{
      calibration_line = line2;
    }
  }
  else if(left_lane_line_intersection.info == 0){
    calibration_line = line1;
  }
  else if(right_lane_line_intersection.info == 0){
    calibration_line = line2;
  }
  else{
    calibration_line = LineABC{};
  }
  return calibration_line;
}


// rotate first, then apply axis offset
// offset = raw_measurement - correct_measurement;
// correct_measurement = raw_measurement - offset;
typedef struct LineCalibrationData{
  float angle_offset;
  float x_axis_offset;
  float y_axis_offset;
}LineCalibrationData;

LineCalibrationData lineCalibration(LineABC measured_line){
  struct LineCalibrationData calibrationData;
  Point2D screen_center;
  Point2D temp_point;
  LineABC rotated_line, temp_line;
  IntersectionLines temp_line_intersection;
  LineABC ideal_line;

  ideal_line = yAxisABC();
  ideal_line.C = -(float)SCREEN_CENTER_X;
  
  calibrationData.angle_offset = -(angleBetweenLinesABC(measured_line, ideal_line));
  calibrationData.x_axis_offset = 0.0f;

  screen_center.x = SCREEN_CENTER_X;
  screen_center.y = SCREEN_CENTER_Y;
  rotated_line = rotateLineAroundPoint(measured_line, screen_center, calibrationData.angle_offset);
  temp_line = perpendicularToLinePassingThroughPointABC(rotated_line, screen_center);
  temp_line_intersection = intersectionLinesABC(temp_line, rotated_line);
  calibrationData.x_axis_offset = temp_line_intersection.point.x - screen_center.x;
  calibrationData.y_axis_offset = temp_line_intersection.point.y - screen_center.y;
  return calibrationData;
}


#endif