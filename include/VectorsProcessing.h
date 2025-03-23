/*
* Copyright 2023 Constantin Dumitru Petre RĂDULICEA
* 
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*   http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef __VECTORSPROCESSING_H__
#define __VECTORSPROCESSING_H__

#include "Arduino.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2Line.h"
#include "geometry2D.h"
#include <vector>

typedef struct FinishLine{
    Vector leftSegment;
    Vector rightSegment;
}FinishLine;

class VectorsProcessing
{
private:
    Vector leftVector;
    Vector rightVector;
    Vector left_calibrated;
    Vector right_calibrated;
    Point2D carPosition;
    float minXaxeAngle;
    float laneWidth;
    LineABC lastMidLine;


    /* data */
public:

    VectorsProcessing(float carPositionX, float carPositionY, float laneWidth, float minXaxeAngle);

    VectorsProcessing(Point2D carPos, float laneWidth, float minXaxeAngle);

    VectorsProcessing();
    
    void setMinXaxisAngle(float minXaxeAngle);

    void setCarPosition(float x, float y);

    void setCarPosition(Point2D carPos);

    void setLaneWidth(float laneWidth);

    bool hasLeftVector();
    bool hasRightVector();

    bool hasMiddleLine();

    void addVector(Vector vec);

    Vector getLeftVector();
    
    Vector getRightVector();

    void setLeftVector(Vector vec);

    void setRightVector(Vector vec);

    LineABC getMiddleLine();
    
    void clear();

    static LineABC vectorToLineABC(Vector vec);

    static float vectorMagnitude(Vector vec1);
    static float vectorAngleWithXaxis(Vector vec1);
    
    ~VectorsProcessing();

    static Vector vectorInvert(Vector vec);
    static Vector mirrorVector(LineABC line, Vector vec);
    static Point2D vectorMidPoint(Vector vec);
    static bool isVectorValid(Vector vec);
    static bool areVectorsEqual(Vector vec1, Vector vec2);
    static bool isFinishLineValid(FinishLine finishLine);
    static LineSegment vectorToLineSegment(Vector vec);

    static Vector lineSegmentToVector(LineSegment seg);
    static float minDistanceVectorToLine(Vector vec, LineABC line);
    static float maxDistanceVectorToLine(Vector vec, LineABC line);
    static Vector reComputeVectorStartEnd_basedOnproximityToPoint(Vector vec, Point2D point);
    static void findIntersections(std::vector<Vector> &vectors, std::vector<Intersection> &intersections);
    static void filterVectorIntersections(std::vector<Vector> &vectors, std::vector<Intersection> &intersections);
    static std::vector<Vector>::iterator findVectorByIndex(std::vector<Vector> &vectors, uint8_t index);
    static void filterVectorIntersection(std::vector<Vector> &vectors, Intersection &intersection);
    static Vector reComputeVectorStartEnd_basedOnDistanceOfPointXaxis(Vector vec, Point2D point);
    static FinishLine findStartFinishLine(std::vector<Vector> &vectors, Vector leftLineVector, Vector rightLineVector, LineABC middleLine, float maxErrorAngleDegrees);
    static int isSecondaryLineCenterInPrimaryCenterRange(LineSegment primary_line, LineSegment secondary_line, float error_angle_rad);
};







#endif