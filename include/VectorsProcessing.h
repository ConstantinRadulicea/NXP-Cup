#ifndef __VECTORSPROCESSING_H__
#define __VECTORSPROCESSING_H__

#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2Line.h"
#include "PurePursuitGeometry.h"
#include "rgb2hsv.h"

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


    /* data */
public:

    VectorsProcessing(float carPositionX, float carPositionY, float laneWidth, float minXaxeAngle){
        this->carPosition.x = carPositionX;
        this->carPosition.y = carPositionY;
        this->minXaxeAngle = minXaxeAngle;
        this->laneWidth = laneWidth;
        this->clear();
    }

    VectorsProcessing(Point2D carPos, float laneWidth, float minXaxeAngle){
        VectorsProcessing(carPos.x, carPos.y, laneWidth, minXaxeAngle);
    }

    VectorsProcessing(){
        this->minXaxeAngle = 0.0f;
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

    void addVector(Vector vec){
        
        if (this->vectorAngleWithXaxis(vec) < this->minXaxeAngle) {
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
        LineABC leftLine, rightLine, middleLine_;
        leftVector_ = this->getLeftVector();
        rightVector_ = this->getRightVector();
        

        if (!this->isVectorValid(leftVector_) && !this->isVectorValid(rightVector_)){
            middleLine_ = yAxisABC();
            middleLine_.C = -this->carPosition.x;
            return middleLine_;
        }

        if (this->isVectorValid(leftVector_)) {
            leftLine = this->vectorToLineABC(leftVector_);
        }

        if (this->isVectorValid(rightVector_)) {
            rightLine = this->vectorToLineABC(rightVector_);
        }

        if (!this->isVectorValid(rightVector_)) {
            rightLine = parallelLineAtDistance(leftLine, this->laneWidth, 1);
        }

        if (!this->isVectorValid(leftVector_)) {
            leftLine = parallelLineAtDistance(rightLine, this->laneWidth, 0);
        }
/*
        leftVector_.print();
        rightVector_.print();

        Serial.println("(" + String(leftLine.Ax) + ")x + " + "(" + String(leftLine.By) + ")y + " + "(" + String(leftLine.C) + ") = 0");
        Serial.println("(" + String(rightLine.Ax) + ")x + " + "(" + String(rightLine.By) + ")y + " + "(" + String(rightLine.C) + ") = 0");
*/
        bisectorsOfTwoLines(leftLine, rightLine, &middleLine_, NULL);
        
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

        point1 = mirrorImage(line, point1);
        point2 = mirrorImage(line, point2);

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

    static bool isVectorValid(Vector &vec){
        if (vec.m_x0 == vec.m_x1 && vec.m_y0 == vec.m_y1)
        {
            return false;
        }
        return true;
    }
};



// return 0 on success
static int aproximateVector(Pixy2& pixy, Vector& vec, float blackTreshold) {
	int8_t res;
    RGBcolor pixel;
	Point2D midPoint_;
	LineABC perpendicularLine, vectorLine;
	int minX, minY, maxX, maxY, vectorMaxX, vectorMaxY;
	int xRight, xLeft, yUp, yDown, xFound, yFound;

    if (!VectorsProcessing::isVectorValid(vec))
    {
        return 0;
    }

    minX = 5;
    minY = 5;
	maxX = pixy.frameWidth;     // 315
	maxY = pixy.frameHeight;    // 207
	vectorMaxX = 78;
	vectorMaxY = 51;

	midPoint_ = VectorsProcessing::vectorMidPoint(vec);
	midPoint_.x = (midPoint_.x / (float)vectorMaxX) * (float)maxX;
	midPoint_.y = (midPoint_.y / (float)vectorMaxY) * (float)maxY;

	vectorLine = VectorsProcessing::vectorToLineABC(vec);
	perpendicularLine = perpendicularToLinePassingThroughPointABC(vectorLine, midPoint_);

	if (!isLineParallelToYaxis(perpendicularLine))
	{
		xRight = (int)roundf(midPoint_.x);
		xLeft = (int)roundf(midPoint_.x-1.0f);
		while (xRight <= maxX || xLeft >= minX) {
			if (xRight <= maxX)
			{
				yFound = (int)roundf(((-perpendicularLine.Ax) * (float)xRight) - perpendicularLine.C);
				if (yFound > maxY || yFound < minY) {
					xRight = maxX + 1;
					continue;
				}
				xFound = xRight;
				xRight++;
                //Serial.println("% right " + String(xFound) + ", " + String(yFound));
                // read pixel and do calculations
                if (pixy.video.getRGB((uint16_t)xFound, (uint16_t)yFound, &(pixel.R), &(pixel.G), &(pixel.B), false) == 0)                {
                    if(rgb2hsv(pixel).V <= blackTreshold){
                        vec.m_x1 = (uint8_t)((xFound / (float)maxX) * (float)vectorMaxX);
                        vec.m_y1 = (uint8_t)((yFound / (float)maxY) * (float)vectorMaxY);
                        break;
                    }
                }else{
                    break;
                }

			}
			if (xLeft >= minX)
			{
				yFound = (int)roundf(((-perpendicularLine.Ax) * (float)xLeft) - perpendicularLine.C);
				if (yFound > maxY || yFound < minY) {
					xLeft = -1;
					continue;
				}
				xFound = xLeft;
				xLeft--;
                //Serial.println("% left " + String(xFound) + ", " + String(yFound));
                // read pixel and do calculations
				if (pixy.video.getRGB((uint16_t)xFound, (uint16_t)yFound, &(pixel.R), &(pixel.G), &(pixel.B), false) == 0)
                {
                    if(rgb2hsv(pixel).V <= blackTreshold){
                        vec.m_x1 = (uint8_t)((xFound / (float)maxX) * (float)vectorMaxX);
                        vec.m_y1 = (uint8_t)((yFound / (float)maxY) * (float)vectorMaxY);
                        break;
                    }
                }else{
                    break;
                }
                
			}
		}
	}
	else {
		yUp = (int)roundf(midPoint_.y);
		yDown = (int)roundf(midPoint_.y - 1.0f);
		xFound = midPoint_.x;
		while (yUp <= maxX || yDown >= minY) {
			if (yUp <= maxX)
			{
				yFound = yUp;
				yUp++;

                // read pixel and do calculations
                if (pixy.video.getRGB((uint16_t)xFound, (uint16_t)yFound, &(pixel.R), &(pixel.G), &(pixel.B), false) == 0)                {
                    if(rgb2hsv(pixel).V <= blackTreshold){
                        vec.m_x1 = (uint8_t)((xFound / (float)maxX) * (float)vectorMaxX);
                        vec.m_y1 = (uint8_t)((yFound / (float)maxY) * (float)vectorMaxY);
                        break;
                    }
                }else{
                    break;
                }
                
			}
			if (yDown >= minY)
			{
				yFound = yDown;
				yDown--;

                // read pixel and do calculations
                if (pixy.video.getRGB((uint16_t)xFound, (uint16_t)yFound, &(pixel.R), &(pixel.G), &(pixel.B), false) == 0)                {
                    if(rgb2hsv(pixel).V <= blackTreshold){
                        vec.m_x1 = (uint8_t)((xFound / (float)maxX) * (float)vectorMaxX);
                        vec.m_y1 = (uint8_t)((yFound / (float)maxY) * (float)vectorMaxY);
                        break;
                    }
                }else{
                    break;
                }
                
			}
		}
	}

    

	return 0;
}





#endif