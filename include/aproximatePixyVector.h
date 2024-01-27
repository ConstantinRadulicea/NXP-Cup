#ifndef __APPROXIMATEPIXYVECTOT_H__
#define __APPROXIMATEPIXYVECTOT_H__

#include "rgb2hsv.h"
#include "VectorsProcessing.h"
#include "PurePursuitGeometry.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h"

static Vector vectorApproximationSelectionLogic(Vector vec, Point2D approximatedPointFound, Point2D carPosition) {
	Point2D startVector, endVector, newVectorStart, newVectorEnd;
	float distanceStartVector, distanceEndVector, distanceApproximatedPointFound;
	Vector newVector;


	memset(&newVector, 0, sizeof(newVector));

	startVector.x = (float)vec.m_x0;
	startVector.y = (float)vec.m_y0;

	endVector.x = (float)vec.m_x1;
	endVector.y = (float)vec.m_y1;

	distanceStartVector = euclidianDistance(carPosition, startVector);
	distanceEndVector = euclidianDistance(carPosition, endVector);
	distanceApproximatedPointFound = euclidianDistance(carPosition, approximatedPointFound);

	if (distanceStartVector < distanceEndVector && distanceStartVector < distanceApproximatedPointFound){
		newVectorStart = startVector;
		newVectorEnd = approximatedPointFound;
	}
	else if (distanceEndVector < distanceStartVector && distanceEndVector < distanceApproximatedPointFound){
		newVectorStart = endVector;
		newVectorEnd = approximatedPointFound;
	}
	else{
		newVectorStart = approximatedPointFound;
		newVectorEnd = endVector;
	}

	newVector.m_x0 = newVectorStart.x;
	newVector.m_y0 = newVectorStart.y;
	newVector.m_x1 = newVectorEnd.x;
	newVector.m_y1 = newVectorEnd.y;

	return newVector;
}

// return 0 on success
static int approximatePixyVectorVector(Pixy2& pixy, Vector& vec, float blackTreshold, Point2D carPosition) {
	int8_t res;
    RGBcolor pixel;
	Point2D midPoint_, pointFound;
	LineABC perpendicularLine, vectorLine;
	int minX, minY, maxX, maxY, vectorMaxX, vectorMaxY;
	int xRight, xLeft, yUp, yDown, xFound, yFound;

    if (!VectorsProcessing::isVectorValid(vec)) {
        return 0;
    }

    minX = 5;
    minY = 5;
	maxX = ((int)pixy.frameWidth);     // 315
	maxY = ((int)pixy.frameHeight);    // 207
	vectorMaxX = 78;
	vectorMaxY = 51;

	midPoint_ = VectorsProcessing::vectorMidPoint(vec);
	midPoint_.x = (midPoint_.x / (float)vectorMaxX) * (float)maxX;
	midPoint_.y = (midPoint_.y / (float)vectorMaxY) * (float)maxY;

	vectorLine = VectorsProcessing::vectorToLineABC(vec);
	perpendicularLine = perpendicularToLinePassingThroughPointABC(vectorLine, midPoint_);

	if (!isLineParallelToYaxis(perpendicularLine)) {
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
						pointFound.x = ((xFound / (float)maxX) * (float)vectorMaxX);
						pointFound.y = ((yFound / (float)maxY) * (float)vectorMaxY);
						vec = vectorApproximationSelectionLogic(vec, pointFound, carPosition);
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
						pointFound.x = ((xFound / (float)maxX) * (float)vectorMaxX);
						pointFound.y = ((yFound / (float)maxY) * (float)vectorMaxY);
						vec = vectorApproximationSelectionLogic(vec, pointFound, carPosition);
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
						pointFound.x = ((xFound / (float)maxX) * (float)vectorMaxX);
						pointFound.y = ((yFound / (float)maxY) * (float)vectorMaxY);
						vec = vectorApproximationSelectionLogic(vec, pointFound, carPosition);
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
						pointFound.x = ((xFound / (float)maxX) * (float)vectorMaxX);
						pointFound.y = ((yFound / (float)maxY) * (float)vectorMaxY);
						vec = vectorApproximationSelectionLogic(vec, pointFound, carPosition);
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
