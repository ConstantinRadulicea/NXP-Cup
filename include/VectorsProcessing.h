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

#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2Line.h"
#include "geometry2D.h"
#include <vector>

class VectorsProcessing
{
private:
    Vector leftVector;
    Vector rightVector;
    Point2D carPosition;
    float minXaxeAngle;
    float laneWidth;
    LineABC lastMidLine;


    /* data */
public:

    VectorsProcessing(float carPositionX, float carPositionY, float laneWidth, float minXaxeAngle){
        this->carPosition.x = carPositionX;
        this->carPosition.y = carPositionY;
        this->minXaxeAngle = minXaxeAngle;
        this->laneWidth = laneWidth;
        this->lastMidLine = yAxisABC();
        this->lastMidLine.C = -this->carPosition.x;
        this->clear();
    }

    VectorsProcessing(Point2D carPos, float laneWidth, float minXaxeAngle){
        VectorsProcessing(carPos.x, carPos.y, laneWidth, minXaxeAngle);
    }

    VectorsProcessing(){
        this->minXaxeAngle = -1.0f;
        this->carPosition.x = 0.0f;
        this->carPosition.y = 0.0f;
        this->laneWidth =  0.0f;
        this->clear();
    }
    
    void setMinXaxisAngle(float minXaxeAngle){
         this->minXaxeAngle = minXaxeAngle;
    }

    void setCarPosition(float x, float y){
        this->carPosition.x = x;
        this->carPosition.y = y;
    }

    void setCarPosition(Point2D carPos){
        this->carPosition = carPos;
    }

    void setLaneWidth(float laneWidth){
        this->laneWidth = laneWidth;
    }

    bool hasLeftVector(){
        return this->isVectorValid(this->leftVector);
    }
    bool hasRightVector(){
        return this->isVectorValid(this->rightVector);
    }

    bool hasMiddleLine(){
        return (this->hasLeftVector() || this->hasLeftVector());
    }

    void addVector(Vector vec){
        
        if (this->minXaxeAngle >= 0.0f && fabs(this->vectorAngleWithXaxis(vec)) < this->minXaxeAngle) {
            return;
        }
        
        /*
        vec.print();
        Serial.println("Carposition: " + String(carPosition.x));
        */
        if (((float)vec.m_x0 >= carPosition.x) && (vectorMagnitude(vec) > vectorMagnitude(this->rightVector))) {
            this->rightVector = vec;
        }
        else if(((float)vec.m_x0 < carPosition.x) && (vectorMagnitude(vec) > vectorMagnitude(this->leftVector))){
            this->leftVector = vec;
        }
    }

    Vector getLeftVector(){
        return this->leftVector;
    }
    
    Vector getRightVector(){
        return this->rightVector;
    }

    void setLeftVector(Vector vec){
        this->leftVector = vec;
    }

    void setRightVector(Vector vec){
        this->rightVector = vec;
    }

    LineABC getMiddleLine(){
        Vector leftVector_, rightVector_;
        LineABC leftLine, rightLine, middleLine_, acuteAngleBisector, ottuseAngleBisector;
        leftVector_ = this->getLeftVector();
        rightVector_ = this->getRightVector();
        

        if (!this->isVectorValid(leftVector_) && !this->isVectorValid(rightVector_)){
            middleLine_ = yAxisABC();
            middleLine_.C = -this->carPosition.x;
            return middleLine_;
            //return lastMidLine;
        }

        if (this->isVectorValid(leftVector_)) {
            leftLine = this->vectorToLineABC(leftVector_);
        }

        if (this->isVectorValid(rightVector_)) {
            rightLine = this->vectorToLineABC(rightVector_);
        }

        if (!this->isVectorValid(rightVector_)) {
            rightLine = parallelLineAtDistanceABC(leftLine, this->laneWidth, 1);
        }

        if (!this->isVectorValid(leftVector_)) {
            leftLine = parallelLineAtDistanceABC(rightLine, this->laneWidth, 0);
        }
/*
        leftVector_.print();
        rightVector_.print();

        Serial.println("(" + String(leftLine.Ax) + ")x + " + "(" + String(leftLine.By) + ")y + " + "(" + String(leftLine.C) + ") = 0");
        Serial.println("(" + String(rightLine.Ax) + ")x + " + "(" + String(rightLine.By) + ")y + " + "(" + String(rightLine.C) + ") = 0");
*/
        bisectorsOfTwoLinesABC(leftLine, rightLine, &acuteAngleBisector, &ottuseAngleBisector);
        if (ottuseAngleBisector.Ax == 0.0f && ottuseAngleBisector.By == 0.0f)
        {
           middleLine_ = acuteAngleBisector;
        }
        else if (fabsf(angleBetweenLinesABC(leftLine, acuteAngleBisector)) < fabsf(angleBetweenLinesABC(leftLine, ottuseAngleBisector)))
        {
            middleLine_ = acuteAngleBisector;
        }
        else{
            middleLine_ = ottuseAngleBisector;
        }
        //lastMidLine = middleLine_;
        return middleLine_;
    }
    
    void clear(){
        memset(&leftVector, 0, sizeof(leftVector));
        memset(&rightVector, 0, sizeof(rightVector));
    }

    static LineABC vectorToLineABC(Vector vec){
        LineABC line2;
        Point2D point1, point2;

        point1.x = (float)vec.m_x0;
        point1.y = (float)vec.m_y0;

        point2.x = (float)vec.m_x1;
        point2.y = (float)vec.m_y1;

        line2 = points2lineABC(point1, point2);
        return line2;
    }

    static float vectorMagnitude(Vector vec1){
        Point2D point1, point2;
        point1.x = (float)vec1.m_x0;
        point1.y = (float)vec1.m_y0;

        point2.x = (float)vec1.m_x1;
        point2.y = (float)vec1.m_y1;

        return euclidianDistance(point1, point2);
    }
    static float vectorAngleWithXaxis(Vector vec1){
        LineABC line2;

        line2 = vectorToLineABC(vec1);
        return angleBetweenLinesABC(xAxisABC(), line2);
    }
    
    
    ~VectorsProcessing(){}

    static Vector vectorInvert(Vector vec){
        Vector vec2;
        vec2 = vec;
        vec2.m_x0 = vec.m_x1;
        vec2.m_y0 = vec.m_y1;
        vec2.m_x1 = vec.m_x0;
        vec2.m_y1 = vec.m_y0;

        return vec2;
    }

    static Vector mirrorVector(LineABC line, Vector vec){
        Point2D point1, point2;

        point1.x = (float)vec.m_x0;
        point1.y = (float)vec.m_y0;

        point2.x = (float)vec.m_x1;
        point2.y = (float)vec.m_y1;

        point1 = mirrorImageABC(line, point1);
        point2 = mirrorImageABC(line, point2);

        vec.m_x0 = (uint8_t)point1.x;
        vec.m_y0 = (uint8_t)point1.y;

        vec.m_x1 = (uint8_t)point2.x;
        vec.m_y1 = (uint8_t)point2.y;
        return vec;
    }
    
    static Point2D vectorMidPoint(Vector vec) {
        Point2D point1, point2;
        point1.x = (float)vec.m_x0;
        point1.y = (float)vec.m_y0;

        point2.x = (float)vec.m_x1;
        point2.y = (float)vec.m_y1;

        return midPoint(point1, point2);
    }

    static bool isVectorValid(Vector vec){
        if (vec.m_x0 == vec.m_x1 && vec.m_y0 == vec.m_y1)
        {
            return false;
        }
        return true;
    }

    static Vector reComputeVectorStartEnd_basedOnproximityToPoint(Vector vec, Point2D point){
        float distanceStartVector, distanceEndVector;
        Point2D startVector, endVector, newVectorStart, newVectorEnd;
        Vector newVector;
        
        startVector.x = (float)vec.m_x0;
        startVector.y = (float)vec.m_y0;

        endVector.x = (float)vec.m_x1;
        endVector.y = (float)vec.m_y1;

        distanceStartVector = euclidianDistance(point, startVector);
	    distanceEndVector = euclidianDistance(point, endVector);

        if (distanceStartVector < distanceEndVector) {
			newVectorStart = startVector;
			newVectorEnd = endVector;
		}
		else{
			newVectorStart = endVector;
			newVectorEnd = startVector;
		}

        newVector.m_x0 = newVectorStart.x;
        newVector.m_y0 = newVectorStart.y;
        newVector.m_x1 = newVectorEnd.x;
        newVector.m_y1 = newVectorEnd.y;
        return newVector;
    }

    static void filterVectorIntersections(std::vector<Vector> &vectors, std::vector<Intersection> &intersections){
        for (size_t i = 0; i < intersections.size(); i++)
        {
            //intersections[i].print();
            filterVectorIntersection(vectors, intersections[i]);
        }
    }

    static std::vector<Vector>::iterator findVectorByIndex(std::vector<Vector> &vectors, uint8_t index){
        for(std::vector<Vector>::iterator it = vectors.begin(); it != vectors.end(); it++){
            if (it->m_index == index) {
                return it;
            }
        }
        return vectors.end();
    }

    static void filterVectorIntersection(std::vector<Vector> &vectors, Intersection &intersection){
        IntersectionLine bestIntersectionLine;
        Vector bestVector;
        Vector newVector;
        std::vector<Vector>::iterator newVectorIterator, bestVectorIterator;

        for (size_t i = 0; i < intersection.m_n; i++)
        {
            newVectorIterator = findVectorByIndex(vectors, intersection.m_intLines[i].m_index);
            if (newVectorIterator == vectors.end()) {
                continue;
            }
            newVector = *newVectorIterator;
            if (i == 0)
            {
                bestIntersectionLine = intersection.m_intLines[i];
                bestVector = newVector;
                bestVectorIterator = newVectorIterator;
            }
            else if(vectorMagnitude(newVector) > vectorMagnitude(bestVector)){
                bestIntersectionLine = intersection.m_intLines[i];
                bestVector = newVector;
                bestVectorIterator = newVectorIterator;

                vectors.erase(bestVectorIterator);
            }
            else{
                vectors.erase(newVectorIterator);
            }
        }
    }

    static Vector reComputeVectorStartEnd_basedOnDistanceOfPointXaxis(Vector vec, Point2D point){
        LineABC pointXaxis;
        float distanceStartVector, distanceEndVector;
        Point2D startVector, endVector, newVectorStart, newVectorEnd;
        Vector newVector;

        pointXaxis = xAxisABC();
        pointXaxis.C -= point.y;

        
        startVector.x = (float)vec.m_x0;
        startVector.y = (float)vec.m_y0;

        endVector.x = (float)vec.m_x1;
        endVector.y = (float)vec.m_y1;

        distanceStartVector = distance2lineABC(startVector, pointXaxis);
        distanceEndVector = distance2lineABC(endVector, pointXaxis);

        if (distanceStartVector < distanceEndVector) {
			newVectorStart = startVector;
			newVectorEnd = endVector;
		}
		else{
			newVectorStart = endVector;
			newVectorEnd = startVector;
		}

        newVector.m_x0 = newVectorStart.x;
        newVector.m_y0 = newVectorStart.y;
        newVector.m_x1 = newVectorEnd.x;
        newVector.m_y1 = newVectorEnd.y;
        return newVector;
    }
};







#endif