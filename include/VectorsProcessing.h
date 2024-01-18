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

    float vectorMagnitude(Vector vec1){
        Point2D point1, point2;
        point1.x = (float)vec1.m_x0;
        point1.y = (float)vec1.m_y0;

        point2.x = (float)vec1.m_x1;
        point2.y = (float)vec1.m_y1;

        return euclidianDistance(point1, point2);
    }
    float vectorAngleWithXaxis(Vector vec1){
        LineMQ xAxe, line2;
        Point2D point1, point2;
        xAxe.m = 0;
        xAxe.q = 0;

        point1.x = (float)vec1.m_x0;
        point1.y = (float)vec1.m_y0;

        point2.x = (float)vec1.m_x1;
        point2.y = (float)vec1.m_y1;

        line2 = points2line(point1, point2);
        return angleBetweenLines(xAxe, line2);
    }

    /* data */
public:

    VectorsProcessing(Point2D carPosition, float minXaxeAngle){
        this->minXaxeAngle = minXaxeAngle;
        this->carPosition = carPosition;
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
    
    void clear(){
        memset(&leftVector, 0, sizeof(leftVector));
        memset(&rightVector, 0, sizeof(rightVector));
    }
    
    
    ~VectorsProcessing(){}
};





#endif