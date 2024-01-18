#ifndef __VECTORSPROCESSING_H__
#define __VECTORSPROCESSING_H__

#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2Line.h"
#include "PurePursuitGeometry.h"


class VectorsProcessing
{
private:
    Vector leftVector;
    Vector rightVector;
    Point2D carPosition;
    float minXaxeAngle;
    float laneWidth;

    float vectorMagnitude(Vector vec1){
        Point2D point1, point2;
        point1.x = (float)vec1.m_x0;
        point1.y = (float)vec1.m_y0;

        point2.x = (float)vec1.m_x1;
        point2.y = (float)vec1.m_y1;

        return euclidianDistance(point1, point2);
    }
    float vectorAngleWithXaxis(Vector vec1){
        LineABC line2;

        line2 = this->vectorToLineABC(vec1);
        return angleBetweenLinesABC(xAxisABC(), line2);
    }
    bool isVectorValid(Vector &vec){
        Vector invalidVector;
        memset(&invalidVector, 0, sizeof(invalidVector));
        if (memcmp(&vec, &invalidVector, sizeof(vec)) == 0){
            return false;
        }
        return true;
    }
    LineABC vectorToLineABC(Vector vec){
        LineABC line2;
        Point2D point1, point2;

        point1.x = (float)vec.m_x0;
        point1.y = (float)vec.m_y0;

        point2.x = (float)vec.m_x1;
        point2.y = (float)vec.m_y1;

        line2 = points2lineABC(point1, point2);
        return line2;
    }
    /* data */
public:

    VectorsProcessing(float carPositionX, float carPositionY, float laneWidth, float minXaxeAngle){
        this->carPosition.x = carPositionX;
        this->carPosition.y = carPositionY;
        this->minXaxeAngle = minXaxeAngle;
        this->laneWidth = laneWidth;
        this->clear();
    }

    VectorsProcessing(){
        this->minXaxeAngle = 0.0f;
        this->carPosition.x = 0;
        this->carPosition.y = 0;
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

    void setLaneWidth(float laneWidth){
        this->laneWidth = laneWidth;
    }

    void addVector(Vector vec){
        if (this->vectorAngleWithXaxis(vec) < this->minXaxeAngle) {
            return;
        }
        
        if (vec.m_x0 >= carPosition.x && vectorMagnitude(vec) > vectorMagnitude(rightVector)) {
            rightVector = vec;
        }
        else if(vectorMagnitude(vec) > vectorMagnitude(leftVector)){
            leftVector = vec;
        }
    }

    Vector getLeftVector(){
        return leftVector;
    }
    
    Vector getRightVector(){
        return rightVector;
    }

    LineABC getMiddleLine(){
        Vector leftVector_, rightVector_;
        LineABC leftLine, rightLine, middleLine_;
        leftVector_ = this->getLeftVector();
        rightVector_ = this->getRightVector();


        if (!this->isVectorValid(leftVector_) && !this->isVectorValid(rightVector_)){
            return yAxisABC();
        }

        if (this->isVectorValid(leftVector_)) {
            leftLine = this->vectorToLineABC(leftVector_);
        }

        if (this->isVectorValid(rightVector_)) {
            rightLine = this->vectorToLineABC(rightVector_);
        }

        if (!this->isVectorValid(rightVector_)) {
            rightLine = leftLine;
            rightLine.C -= this->laneWidth;
        }

        if (!this->isVectorValid(leftVector_)) {
            leftLine = rightLine;
            leftLine.C -= this->laneWidth;
        }

        middleLine_ = middleLineABC(leftLine, rightLine);
        
        return middleLine_;
    }
    
    void clear(){
        memset(&leftVector, 0, sizeof(leftVector));
        memset(&rightVector, 0, sizeof(rightVector));
    }
    
    
    ~VectorsProcessing(){}
};





#endif