#ifndef __VECTORSPROCESSING_H__
#define __VECTORSPROCESSING_H__

#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2Line.h"
#include "PurePursuitGeometry.h"


class VectorsProcessing
{
private:
    Vector leftVector;
    Vector rightVector;

    float angleWithXaxis(Vector vec1){
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
    VectorsProcessing(/* args */){}
    void addVector(Vector vec){

    }

    Vector getLeftVector(){
        return leftVector;
    }
    Vector getRightVector(){
        return rightVector;
    }
    ~VectorsProcessing(){}
};





#endif