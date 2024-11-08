#include <stdint.h>
#include <string>

#include "TrajectoryMix.h"

int main() {
	LineCalibrationData calibration_data;
	LineABC measured_line;
	LineABC temp_line_1;
	LineSegment measured_segment, calibrated_segment;

	measured_segment.A = Point2D{ 36.0f, 0.0f };
	measured_segment.B = Point2D{ 29.6f, 40.0f };

	measured_line = lineSegmentToLineABC(measured_segment);
	calibration_data = lineCalibration(measured_line);
	calibrated_segment = calibrateLineSegment(measured_segment, calibration_data);
	temp_line_1 = lineSegmentToLineABC(calibrated_segment);

	float ggg = std::stof("++2.0 ");

	return 0;
}