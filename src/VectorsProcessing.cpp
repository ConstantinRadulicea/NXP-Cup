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


#include "VectorsProcessing.h"
#include "GlobalVariables.h"


struct TrackLines {
	LineSegment left_line;
	LineSegment right_line;
};

/*	1 week before finals, 8.2 run, hard left in intersection fix (maybe) */
struct TrackLines intersectingCornerLinesFiltering (LineSegment _left_line, LineSegment _right_line, LineABC Horizontal_car_line, LineABC vertical_car_line) {
	struct TrackLines result;
	//Point2D carPosition = { 35.0f, 0.0f };
	//int temp_int, temp_int2;
	IntersectionLines line_segments_intersection;

	//LineABC Horizontal_car_line = xAxisABC();
	//Horizontal_car_line.C = -carPosition.y;

	memset(&result, 0, sizeof(result));
	result.left_line = _left_line;
	result.right_line = _right_line;

	if (isValidLineSegment(_left_line) == 0 || isValidLineSegment(_right_line) == 0) {
		return result;
	}

	if (areLineSegmentsEqual(_left_line, _right_line) != 0) {
		return result;
	}

	LineSegment seg1_to_horizontal_line = getLineSegmentFromStartPointAToLine(_left_line, Horizontal_car_line);
	LineSegment seg2_to_horizontal_line = getLineSegmentFromStartPointAToLine(_right_line, Horizontal_car_line);


	if (isValidLineSegment(seg1_to_horizontal_line) == 0) {
		// do something about it
		return result;
	}

	if (isValidLineSegment(seg2_to_horizontal_line) == 0) {
		// do something about it
		return result;
	}

	LineSegment seg1_B_to_horizontal_line = seg1_to_horizontal_line;
	seg1_B_to_horizontal_line.A = _left_line.B;

	LineSegment seg2_B_to_horizontal_line = seg2_to_horizontal_line;
	seg2_B_to_horizontal_line.A = _right_line.B;

	line_segments_intersection = lineSegmentIntersection(seg1_B_to_horizontal_line, seg2_B_to_horizontal_line);


	if (line_segments_intersection.info != INTERSECTION_INFO_ONE_INTERSECTION) {
		return result;
	}

	// check special case

	IntersectionLines seg1_to_car_vertical;
	IntersectionLines seg2_to_car_vertical;


	seg1_to_car_vertical = intersectionLinesABC(vertical_car_line, lineSegmentToLineABC(seg1_to_horizontal_line));
	seg2_to_car_vertical = intersectionLinesABC(vertical_car_line, lineSegmentToLineABC(seg2_to_horizontal_line));


	if (seg1_to_car_vertical.info == INTERSECTION_INFO_ONE_INTERSECTION) {
		if (isPointOnSegment(seg1_to_horizontal_line, seg1_to_car_vertical.point)!= 0) {
			memset(&(result.left_line), 0, sizeof(LineSegment));
		}
	}

	if (seg2_to_car_vertical.info == INTERSECTION_INFO_ONE_INTERSECTION) {
		if (isPointOnSegment(seg2_to_horizontal_line, seg2_to_car_vertical.point) != 0) {
			memset(&(result.right_line), 0, sizeof(LineSegment));
		}
		
	}

	return result;
}






VectorsProcessing::VectorsProcessing(float carPositionX, float carPositionY, float laneWidth, float minXaxeAngle){
        this->carPosition.x = carPositionX;
        this->carPosition.y = carPositionY;
        this->minXaxeAngle = minXaxeAngle;
        this->laneWidth = laneWidth;
        this->lastMidLine = yAxisABC();
        this->lastMidLine.C = -this->carPosition.x;
        this->clear();
        memset(&(this->leftVector), 0, sizeof(this->leftVector));
        memset(&(this->rightVector), 0, sizeof(this->leftVector));
    }

    VectorsProcessing::VectorsProcessing(Point2D carPos, float laneWidth, float minXaxeAngle){
        VectorsProcessing(carPos.x, carPos.y, laneWidth, minXaxeAngle);
    }

    VectorsProcessing::VectorsProcessing(){
        this->minXaxeAngle = -1.0f;
        this->carPosition.x = 0.0f;
        this->carPosition.y = 0.0f;
        this->laneWidth =  0.0f;
        this->clear();
    }
    
    void VectorsProcessing::setMinXaxisAngle(float minXaxeAngle){
         this->minXaxeAngle = minXaxeAngle;
    }

    void VectorsProcessing::setCarPosition(float x, float y){
        this->carPosition.x = x;
        this->carPosition.y = y;
    }

    void VectorsProcessing::setCarPosition(Point2D carPos){
        this->carPosition = carPos;
    }

    void VectorsProcessing::setLaneWidth(float laneWidth){
        this->laneWidth = laneWidth;
    }

    bool VectorsProcessing::hasLeftVector(){
        return isValidLineSegment(this->leftVector);
    }
    bool VectorsProcessing::hasRightVector(){
        return isValidLineSegment(this->rightVector);
    }

    bool VectorsProcessing::hasMiddleLine(){
        return (this->hasLeftVector() || this->hasLeftVector());
    }

    void VectorsProcessing::addVector(LineSegment vec){
        if (!isValidLineSegment(vec)) {
            return;
        }

        if (lengthLineSegment(vec) < MeterToVectorUnit(0.15f)) {
            return;
        }
        
        if (this->minXaxeAngle >= 0.0f && fabs(this->lineSegmentAngleWithXaxis(vec)) < this->minXaxeAngle) {
            return;
        }
        
        /*
        vec.print();
        Serial.println("Carposition: " + String(carPosition.x));
        */

        LineABC new_vect_line = lineSegmentToLineABC(vec);
        LineABC Horizontal_car_line = xAxisABC();
        Horizontal_car_line.C = -carPosition.y;
        IntersectionLines inters;
        int temp_int, temp_int2;

        inters = intersectionLinesABC(Horizontal_car_line, new_vect_line);
        if (inters.info != INTERSECTION_INFO_ONE_INTERSECTION) {
            return;
        }
        
        //inters.point.x = vec.m_x0;
        //inters.point.y = vec.m_y0;

        if ((float)inters.point.x >= carPosition.x) {
            temp_int = isReachableSegment(carPosition, this->leftVector, vec);
            
            if (temp_int) {
                this->rightVector = getLongestReachableSegment(carPosition, this->rightVector, vec);
                temp_int2 = isReachableSegment(carPosition, this->rightVector, this->leftVector);
                if(temp_int2 == 0){
                    memset(&(this->leftVector), 0, sizeof(this->leftVector));
                }
            }

            //this->rightVector = vec;
        }
        else if(inters.point.x < carPosition.x){
            temp_int = isReachableSegment(carPosition, this->rightVector, vec);
            if (temp_int)
            {
                this->leftVector = getLongestReachableSegment(carPosition, this->leftVector, vec);
                temp_int2 = isReachableSegment(carPosition, this->leftVector, this->rightVector);
                if(temp_int2 == 0){
                    memset(&(this->rightVector), 0, sizeof(this->rightVector));
                }
            }
            
            //this->leftVector = vec;
        }
        
    }

    LineSegment VectorsProcessing::getLeftVector(){
        float left_line_size, right_line_size;
        if (isValidLineSegment(this->leftVector) && isValidLineSegment(this->rightVector)) {
            left_line_size = lengthLineSegment(this->leftVector);
            right_line_size = lengthLineSegment(this->rightVector);
            if (left_line_size >= right_line_size) {
                return this->leftVector;
            }
            else if(isSecondaryLineCenterInPrimaryCenterRange(this->rightVector, this->leftVector, radians(45.0f)) == 0){
                memset(&(this->leftVector), 0, sizeof(this->leftVector));
            }
        }


        struct TrackLines double_checked_lines;

        LineABC Horizontal_car_line = xAxisABC();
        Horizontal_car_line.C = -carPosition.y;

        LineABC Vertical_car_line = yAxisABC();
        Vertical_car_line.C = -carPosition.x;


        double_checked_lines = intersectingCornerLinesFiltering(this->leftVector, this->rightVector, Horizontal_car_line, Vertical_car_line);
        
        this->leftVector = double_checked_lines.left_line;

        return this->leftVector;
    }
    
    LineSegment VectorsProcessing::getRightVector(){
        float left_line_size, right_line_size;
        if (isValidLineSegment(this->leftVector) && isValidLineSegment(this->rightVector)) {
            left_line_size = lengthLineSegment(this->leftVector);
            right_line_size = lengthLineSegment(this->rightVector);
            if (right_line_size > left_line_size) {
                return this->rightVector;
            }
            else if(isSecondaryLineCenterInPrimaryCenterRange((this->leftVector), (this->rightVector), radians(45.0f)) == 0){
                memset(&(this->rightVector), 0, sizeof(this->rightVector));
            }
        }


        struct TrackLines double_checked_lines;

        LineABC Horizontal_car_line = xAxisABC();
        Horizontal_car_line.C = -carPosition.y;

        LineABC Vertical_car_line = yAxisABC();
        Vertical_car_line.C = -carPosition.x;


        double_checked_lines = intersectingCornerLinesFiltering(this->leftVector, this->rightVector, Horizontal_car_line, Vertical_car_line);
        
        this->rightVector = double_checked_lines.right_line;

        return this->rightVector;
        

        return this->rightVector;
    }

    void VectorsProcessing::setLeftVector(LineSegment vec){
        this->leftVector = vec;
    }

    void VectorsProcessing::setRightVector(LineSegment vec){
        this->rightVector = vec;
    }

    LineABC VectorsProcessing::getMiddleLine(){
        LineSegment leftVector_, rightVector_;
        LineABC leftLine, rightLine, middleLine_, acuteAngleBisector, ottuseAngleBisector;
        leftVector_ = this->getLeftVector();
        rightVector_ = this->getRightVector();

        leftLine = yAxisABC();
        rightLine = yAxisABC();
        

        if (!isValidLineSegment(leftVector_) && !isValidLineSegment(rightVector_)){
            middleLine_ = yAxisABC();
            middleLine_.C = -this->carPosition.x;
            return middleLine_;
        }

        if (isValidLineSegment(leftVector_)) {
            leftLine = lineSegmentToLineABC(leftVector_);
        }

        if (isValidLineSegment(rightVector_)) {
            rightLine = lineSegmentToLineABC(rightVector_);
        }

        if (!isValidLineSegment(rightVector_)) {
            rightLine = parallelLineAtDistanceABC(leftLine, this->laneWidth, 1);
        }

        if (!isValidLineSegment(leftVector_)) {
            leftLine = parallelLineAtDistanceABC(rightLine, this->laneWidth, 0);
        }
/*
        leftVector_.print();
        rightVector_.print();

        Serial.println("(" + String(leftLine.Ax) + ")x + " + "(" + String(leftLine.By) + ")y + " + "(" + String(leftLine.C) + ") = 0");
        Serial.println("(" + String(rightLine.Ax) + ")x + " + "(" + String(rightLine.By) + ")y + " + "(" + String(rightLine.C) + ") = 0");
*/
        bisectorsOfTwoLinesABC(leftLine, rightLine, &acuteAngleBisector, &ottuseAngleBisector);
        if ((floatCmp(ottuseAngleBisector.Ax, 0.0f) == 0) && (floatCmp(ottuseAngleBisector.By, 0.0f) == 0))
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
    
    void VectorsProcessing::clear(){
        memset(&leftVector, 0, sizeof(leftVector));
        memset(&rightVector, 0, sizeof(rightVector));
    }

    LineABC VectorsProcessing::vectorToLineABC(Vector vec){
        LineABC line2;
        Point2D point1, point2;

        point1.x = (float)vec.m_x0;
        point1.y = (float)vec.m_y0;

        point2.x = (float)vec.m_x1;
        point2.y = (float)vec.m_y1;

        line2 = points2lineABC(point1, point2);
        return line2;
    }

    float VectorsProcessing::vectorMagnitude(Vector vec1){
        Point2D point1, point2;
        point1.x = (float)vec1.m_x0;
        point1.y = (float)vec1.m_y0;

        point2.x = (float)vec1.m_x1;
        point2.y = (float)vec1.m_y1;

        return euclidianDistance(point1, point2);
    }
    float VectorsProcessing::lineSegmentAngleWithXaxis(LineSegment vec1){
        LineABC line2;

        line2 = lineSegmentToLineABC(vec1);
        return angleBetweenLinesABC(xAxisABC(), line2);
    }
    
    
    VectorsProcessing::~VectorsProcessing(){}

    Vector VectorsProcessing::vectorInvert(Vector vec){
        Vector vec2;
        vec2 = vec;
        vec2.m_x0 = vec.m_x1;
        vec2.m_y0 = vec.m_y1;
        vec2.m_x1 = vec.m_x0;
        vec2.m_y1 = vec.m_y0;

        return vec2;
    }

    Vector VectorsProcessing::mirrorVector(LineABC line, Vector vec){
        Point2D point1, point2;

        point1.x = (float)vec.m_x0;
        point1.y = (float)vec.m_y0;

        point2.x = (float)vec.m_x1;
        point2.y = (float)vec.m_y1;

        point1 = mirrorImageABC(line, point1);
        point2 = mirrorImageABC(line, point2);

        vec.m_x0 = (int)point1.x;
        vec.m_y0 = (int)point1.y;

        vec.m_x1 = (int)point2.x;
        vec.m_y1 = (int)point2.y;
        return vec;
    }
    
    Point2D VectorsProcessing::vectorMidPoint(Vector vec) {
        Point2D point1, point2;
        point1.x = (float)vec.m_x0;
        point1.y = (float)vec.m_y0;

        point2.x = (float)vec.m_x1;
        point2.y = (float)vec.m_y1;

        return midPoint(point1, point2);
    }

    bool VectorsProcessing::isVectorValid(Vector vec){
        if (vec.m_x0 == vec.m_x1 && vec.m_y0 == vec.m_y1)
        {
            return false;
        }
        return true;
    }

    bool VectorsProcessing::areVectorsEqual(Vector vec1, Vector vec2){
        if (vec1.m_x0 == vec2.m_x0 &&
            vec1.m_y0 == vec2.m_y0 &&
            vec1.m_x1 == vec2.m_x1 &&
            vec1.m_y1 == vec2.m_y1
            ) {
            return true;
        }
        if (vec1.m_x0 == vec2.m_x1 &&
            vec1.m_y0 == vec2.m_y1 &&
            vec1.m_x1 == vec2.m_x0 &&
            vec1.m_y1 == vec2.m_y0
            ) {
            return true;
        }
        return false;
    }

    bool VectorsProcessing::isFinishLineValid(FinishLine finishLine){
        if(isValidLineSegment(finishLine.leftSegment) || isValidLineSegment(finishLine.rightSegment)){
            return true;
        }
        return false;
    }

    LineSegment VectorsProcessing::vectorToLineSegment(Vector vec){
        return LineSegment{Point2D{(float)(vec.m_x0), (float)(vec.m_y0)}, Point2D{(float)(vec.m_x1), (float)(vec.m_y1)}};
    }


    Vector VectorsProcessing::lineSegmentToVector(LineSegment seg){
        Vector vec;
        vec.m_x0 = (int)seg.A.x;
        vec.m_y0 = (int)seg.A.y;

        vec.m_x1 = (int)seg.B.x;
        vec.m_y1 = (int)seg.B.y;

        return vec;
    }

    float VectorsProcessing::minDistanceVectorToLine(Vector vec, LineABC line){
        return minDistanceLineSegmentToLine(vectorToLineSegment(vec), line);
    }

    float VectorsProcessing::maxDistanceVectorToLine(Vector vec, LineABC line){
        return maxDistanceLineSegmentToLine(vectorToLineSegment(vec), line);
    }

    Vector VectorsProcessing::reComputeVectorStartEnd_basedOnproximityToPoint(Vector vec, Point2D point){
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

    void VectorsProcessing::findIntersections(std::vector<Vector> &vectors, std::vector<Intersection> &intersections){
        //intersections.clear();
        Intersection inters;
        std::vector<Vector> vectors_copy = vectors;
        bool isIntersection;
        int x, y;
        for (size_t i = 0; i < (vectors_copy.size()/2); i++)
        {
            x = vectors_copy[i].m_x0;
            y = vectors_copy[i].m_y0;
            memset(&inters, 0, sizeof(Intersection));
            inters.m_x = x;
            inters.m_y = y;
            inters.m_intLines[inters.m_n].m_index = vectors_copy[i].m_index;
            inters.m_n += 1;
            isIntersection = false;
            for (size_t j = i+1; j < vectors_copy.size(); j++) {
                
                if ((euclidianDistance(Point2D{(float)x, (float)y}, Point2D{(float)vectors_copy[j].m_x0, (float)vectors_copy[j].m_y0}) < 5.0f) ||
                (euclidianDistance(Point2D{(float)x, (float)y}, Point2D{(float)vectors_copy[j].m_x1, (float)vectors_copy[j].m_y1}) < 5.0f)) {
                    isIntersection = true;
                    inters.m_x = x;
                    inters.m_y = y;
                    inters.m_intLines[inters.m_n].m_index = vectors_copy[j].m_index;
                    inters.m_n += 1;
                    vectors_copy.erase(vectors_copy.begin() + j);
                    j -= 1;
                }
            }
            if (isIntersection == true) {
                intersections.push_back(inters);
            }

            x = vectors_copy[i].m_x1;
            y = vectors_copy[i].m_y1;
            memset(&inters, 0, sizeof(Intersection));
            inters.m_x = x;
            inters.m_y = y;
            inters.m_intLines[inters.m_n].m_index = vectors_copy[i].m_index;
            inters.m_n += 1;
            isIntersection = false;
            for (size_t j = i+1; j < vectors_copy.size(); j++) {
                if ((euclidianDistance(Point2D{(float)x, (float)y}, Point2D{(float)vectors_copy[j].m_x0, (float)vectors_copy[j].m_y0}) < 5.0f) ||
                (euclidianDistance(Point2D{(float)x, (float)y}, Point2D{(float)vectors_copy[j].m_x1, (float)vectors_copy[j].m_y1}) < 5.0f)) {
                    isIntersection = true;
                    inters.m_x = x;
                    inters.m_y = y;
                    inters.m_intLines[inters.m_n].m_index = vectors_copy[j].m_index;
                    inters.m_n += 1;
                    vectors_copy.erase(vectors_copy.begin() + j);
                    j -= 1;
                }
            }
            if (isIntersection == true) {
                intersections.push_back(inters);
            }
        }
    }

    void VectorsProcessing::filterVectorIntersections(std::vector<Vector> &vectors, std::vector<Intersection> &intersections){
        for (size_t i = 0; i < intersections.size(); i++)
        {
            //intersections[i].print();
            filterVectorIntersection(vectors, intersections[i]);
        }
    }

    std::vector<Vector>::iterator VectorsProcessing::findVectorByIndex(std::vector<Vector> &vectors, uint8_t index){
        for(std::vector<Vector>::iterator it = vectors.begin(); it != vectors.end(); it++){
            if (it->m_index == index) {
                return it;
            }
        }
        return vectors.end();
    }

    void VectorsProcessing::filterVectorIntersection(std::vector<Vector> &vectors, Intersection &intersection){
        IntersectionLine bestIntersectionLine;
        Vector bestVector;
        Vector newVector;
        std::vector<Vector>::iterator newVectorIterator, bestVectorIterator;

        memset(&bestVector, 0, sizeof(Vector));

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

    Vector VectorsProcessing::reComputeVectorStartEnd_basedOnDistanceOfPointXaxis(Vector vec, Point2D point){
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

    FinishLine VectorsProcessing::findStartFinishLine(std::vector<LineSegment> &vectors, LineSegment leftLineVector, LineSegment rightLineVector, LineABC middleLine, float maxErrorAngleDegrees){
        FinishLine finishLine;
        LineABC leftLine, rightLine, tempVectorLine;
        float minDistanceVectorToLeftLine, minDistanceVectorToRightLine, angleRadiansError, angleRadiansError_prev;
        LineSegment leftLine_segment;
        LineSegment rightLine_segment;
        LineSegment vec_segment;
        //float laneWidth_max, laneWidth_min;
        //LineSegmentsDistancePoints laneWidth_;

        memset(&finishLine, 0, sizeof(FinishLine));
        if(!isValidLineSegment(leftLineVector) || !isValidLineSegment(rightLineVector)){
            return finishLine;
        }

        leftLine_segment = leftLineVector;
        rightLine_segment = rightLineVector;
        
        //laneWidth_ = distancePointsBwSegments(vectorToLineSegment(leftLineVector), vectorToLineSegment(rightLineVector));
        //laneWidth_min = euclidianDistance(laneWidth_.min.A, laneWidth_.min.B);
        //laneWidth_max = euclidianDistance(laneWidth_.max.A, laneWidth_.max.B);
        //Serial1.print("%");
        //Serial1.println(laneWidth_min);

        leftLine = lineSegmentToLineABC(leftLineVector);
        rightLine = lineSegmentToLineABC(rightLineVector);

        if (floatCmp(fabs(angleBetweenLinesABC(leftLine, rightLine)), radians(45.0f)) >= 0) {
            return finishLine;
        }

        //if (floatCmp(laneWidth_min, 0.0f) <= 0) {
        //    return finishLine;
        //}

        for (size_t i = 0; i < vectors.size(); i++) {
            if (lengthLineSegment(vectors[i]) > MeterToVectorUnit(0.15)){
                continue;
            }
            if (lengthLineSegment(vectors[i]) < MeterToVectorUnit(0.05)){
                continue;
            }

            if (areLineSegmentsEqual(vectors[i], leftLineVector) || areLineSegmentsEqual(vectors[i], rightLineVector)) {
                continue;
            }

            vec_segment = vectors[i];

            tempVectorLine = lineSegmentToLineABC(vectors[i]);

            angleRadiansError = fabs((M_PI_2 - fabs(angleBetweenLinesABC(middleLine, tempVectorLine))));
            //angleRadiansError = angleBetweenLinesABC(middleLine, tempVectorLine);

            if (floatCmp(angleRadiansError, radians(fabs(maxErrorAngleDegrees))) <= 0) {
                //Serial1.print("%");
                //Serial1.println(angleRadiansError);
                minDistanceVectorToLeftLine = minDistanceLineSegmentToLine(vectors[i], leftLine);
                minDistanceVectorToRightLine = minDistanceLineSegmentToLine(vectors[i], rightLine);

                //if ((floatCmp(minDistanceVectorToLeftLine, 0.0f) <= 0) || (floatCmp(minDistanceVectorToRightLine, 0.0f) <= 0)) {
                //    continue;
                // }
                //Serial1.print("%");
                //Serial1.println(minDistanceVectorToLeftLine);
                //Serial1.print("%");
                //Serial1.println(minDistanceVectorToRightLine);
                //if ((floatCmp(minDistanceVectorToLeftLine, laneWidth_min) > 0) || (floatCmp(minDistanceVectorToRightLine, laneWidth_min) > 0)) {
                //    continue;
                //}

                //if (floatCmp((minDistanceVectorToLeftLine + minDistanceVectorToRightLine), laneWidth_min) >= 0) {
                //    continue;
                //}

                if (isPointInQuadrilateral(leftLine_segment.A, leftLine_segment.B, rightLine_segment.B, rightLine_segment.A, vec_segment.A) == 0) {
                    continue;
                }
                if (isPointInQuadrilateral(leftLine_segment.A, leftLine_segment.B, rightLine_segment.B, rightLine_segment.A, vec_segment.B) == 0) {
                    continue;
                }
                

                if (floatCmp(minDistanceVectorToLeftLine, minDistanceVectorToRightLine) <= 0) {
                    if (isValidLineSegment(finishLine.leftSegment)) {
                        angleRadiansError_prev = fabs((M_PI_2 - fabs(angleBetweenLinesABC(middleLine, lineSegmentToLineABC(finishLine.leftSegment)))));
                        if (floatCmp(angleRadiansError, angleRadiansError_prev) < 0) {
                            finishLine.leftSegment = vectors[i];
                        }
                    }
                    else{
                        finishLine.leftSegment = vectors[i];
                    }
                }

                if (floatCmp(minDistanceVectorToLeftLine, minDistanceVectorToRightLine) > 0) {
                    if (isValidLineSegment(finishLine.rightSegment)) {
                        angleRadiansError_prev = fabs((M_PI_2 - fabs(angleBetweenLinesABC(middleLine, lineSegmentToLineABC(finishLine.rightSegment)))));
                        if (floatCmp(angleRadiansError, angleRadiansError_prev) < 0) {
                            finishLine.rightSegment = vectors[i];
                        }
                    }
                    else{
                        finishLine.rightSegment = vectors[i];
                    }
                }
            }
        }

        return finishLine;
    }


    int  VectorsProcessing::isSecondaryLineCenterInPrimaryCenterRange(LineSegment primary_line, LineSegment secondary_line, float error_angle_rad){
        Point2D primary_center, secondary_center;
        float angle1, angle2, min_angle;
        float lateral_angle_trim;
        error_angle_rad = fabsf(error_angle_rad);
        lateral_angle_trim = M_PI_2 - error_angle_rad;

        primary_center = midPointLineSegment(primary_line);
        secondary_center = midPointLineSegment(secondary_line);

        
        angle1 = angleBw3Points2D(primary_center, secondary_center, primary_line.A);
        angle2 = angleBw3Points2D(primary_center, secondary_center, primary_line.B);
        min_angle = MIN(angle1, angle2);
        //SERIAL_PORT.println("% angle1: " + String(degrees(angle1)));
        //SERIAL_PORT.println("% angle2: " + String(degrees(angle2)));
        //SERIAL_PORT.println("% angle2: " + String(degrees(angle2)));
        return (int)(min_angle >= lateral_angle_trim);
    }

