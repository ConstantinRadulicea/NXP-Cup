#include "TrajectoryMix.h"

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


LineCalibrationData lineCalibration(LineABC measured_line){
  struct LineCalibrationData calibrationData;
  Point2D screen_center;
  Point2D temp_point;
  LineABC rotated_line, temp_line;
  IntersectionLines temp_line_intersection;
  LineABC ideal_line;

  ideal_line = yAxisABC();
  ideal_line.C = -(float)SCREEN_CENTER_X;
  
  calibrationData.angle_offset = (angleBetweenLinesABC(measured_line, ideal_line));
 
  screen_center.x = SCREEN_CENTER_X;
  screen_center.y = SCREEN_CENTER_Y;
  calibrationData.rotation_point = screen_center;
  rotated_line = rotateLineAroundPoint(measured_line, screen_center, -(calibrationData.angle_offset));
  temp_line = perpendicularToLinePassingThroughPointABC(rotated_line, screen_center);
  temp_line_intersection = intersectionLinesABC(temp_line, rotated_line);
  calibrationData.x_axis_offset = temp_line_intersection.point.x - screen_center.x;
  calibrationData.y_axis_offset = temp_line_intersection.point.y - screen_center.y;
  //SERIAL_PORT.println(String("% ") + String(temp_line_intersection.point.x) + String(" ") + String(temp_line_intersection.point.y));
  //SERIAL_PORT.println(String("% ") + String(calibrationData.x_axis_offset) + String(" ") + String(calibrationData.y_axis_offset));

  return calibrationData;
}

//Vector calibrateVector(Vector vec, LineCalibrationData calibration_data){
//  LineSegment seg, calibrated_seg;
//  Vector calibrated_vec;
//  seg = VectorsProcessing::vectorToLineSegment(vec);
//  calibrated_seg = rotateLineSegmentAroundPoint(seg, calibration_data.rotation_point, -(calibration_data.angle_offset));
//  //calibrated_seg.A.x -= calibration_data.x_axis_offset;
//  //calibrated_seg.A.y -= calibration_data.y_axis_offset;
////
//  //calibrated_seg.B.x -= calibration_data.x_axis_offset;
//  //calibrated_seg.B.y -= calibration_data.y_axis_offset;
//
//  calibrated_vec = VectorsProcessing::lineSegmentToVector(calibrated_seg);
//  return calibrated_vec;
//}


LineSegment calibrateLineSegment(LineSegment seg, LineCalibrationData calibration_data) {
    LineSegment calibrated_seg;
    calibrated_seg = rotateLineSegmentAroundPoint(seg, calibration_data.rotation_point, -(calibration_data.angle_offset));
    calibrated_seg.A.x -= calibration_data.x_axis_offset;
    calibrated_seg.A.y -= calibration_data.y_axis_offset;
  
    calibrated_seg.B.x -= calibration_data.x_axis_offset;
    calibrated_seg.B.y -= calibration_data.y_axis_offset;
    return calibrated_seg;
}


//Vector uncalibrateVector(Vector vec, LineCalibrationData calibration_data){
//  LineSegment seg, calibrated_seg;
//  Vector calibrated_vec;
//  seg = VectorsProcessing::vectorToLineSegment(vec);
//  calibrated_seg = rotateLineSegmentAroundPoint(seg, calibration_data.rotation_point, -(calibration_data.angle_offset));
//  calibrated_seg.A.x += calibration_data.x_axis_offset;
//  calibrated_seg.A.y += calibration_data.y_axis_offset;
//
//  calibrated_seg.B.x += calibration_data.x_axis_offset;
//  calibrated_seg.B.y += calibration_data.y_axis_offset;
//
//  calibrated_vec = VectorsProcessing::lineSegmentToVector(calibrated_seg);
//  return calibrated_vec;
//}


