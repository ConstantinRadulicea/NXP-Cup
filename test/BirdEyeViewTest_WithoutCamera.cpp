#include <Arduino.h>
#include "BirdEyeView.h"
#include "TrajectoryMix.h"
#include "GlobalVariables.h"

void setup(){
    Serial.begin(SERIAL_PORT_BAUD_RATE);
    while (!Serial) {delay(100); }
    initialize_g_birdeye_calibrationdata();
    
}


LineSegment calibrated_vector = {};
LineSegment uncalibrated_vector = {};

void loop(){

    for (int y = 0; y < g_line_image_frame_height+1; y++)
    {
        /* code */

        for (int x = 0; x < g_line_image_frame_width+1; x++)
        {
            uncalibrated_vector.B.x = x;
            uncalibrated_vector.B.y = y;
            calibrated_vector = uncalibrated_vector;

            if (g_birdeye_calibrationdata.valid && g_start_line_calibration_acquisition == 0){
                calibrated_vector = BirdEye_CalibrateLineSegmentScaledToVector(g_birdeye_calibrationdata, calibrated_vector);
            }
            if (g_start_line_calibration_acquisition == 0) {
                //calibrated_vector = calibrateVector(calibrated_vector, g_line_calibration_data);
            }
            Serial.print(String(x));
            Serial.print(";");
            Serial.print(String(y));
            Serial.print(";");
            Serial.print(String(calibrated_vector.B.x));
            Serial.print(";");
            Serial.print(String(calibrated_vector.B.y));
            Serial.println();
        }
    }

    while (!Serial.available()) {
        delay(100);
    }
    
    while (Serial.available()) {
        Serial.read();
    }
}




void loop2(){
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            Serial.print(String(g_birdeye_calibrationdata.birdeye_transform_matrix[i][j], 10));
            Serial.print("\t");
        }
        Serial.println();
    }



    LineSegment calibrated_vector, uncalibrated_vector;
    Vector calibrated_vector_vec, uncalibrated_vector_vec;

    uncalibrated_vector.A.x = 0;
    uncalibrated_vector.A.y = 0;

    uncalibrated_vector.B.x = 1;
    uncalibrated_vector.B.y = 7;

    uncalibrated_vector_vec = VectorsProcessing::lineSegmentToVector(uncalibrated_vector);
    calibrated_vector = BirdEye_CalibrateLineSegmentScaledToVector(g_birdeye_calibrationdata, uncalibrated_vector);

    calibrated_vector_vec = BirdEye_CalibrateVector(g_birdeye_calibrationdata, uncalibrated_vector_vec);

    Serial.print(String(uncalibrated_vector.B.x));
    Serial.print(";");
    Serial.print(String(uncalibrated_vector.B.y));
    Serial.print(";");
    Serial.print(String(calibrated_vector.B.x));
    Serial.print(";");
    Serial.print(String(calibrated_vector.B.y));
    Serial.println();

    Serial.print(String(uncalibrated_vector_vec.m_x1));
    Serial.print(";");
    Serial.print(String(uncalibrated_vector_vec.m_y1));
    Serial.print(";");
    Serial.print(String(calibrated_vector_vec.m_x1));
    Serial.print(";");
    Serial.print(String(calibrated_vector_vec.m_y1));
    Serial.println();


    while (!Serial.available()) {
        delay(100);
    }
    
    while (Serial.available()) {
        Serial.read();
    }
    
}