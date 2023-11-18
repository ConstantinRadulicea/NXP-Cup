#ifndef _TRACKLANE_H_
#define _TRACKLANE_H_

#include "PixelGreyscaleRow.h"

class TrackLane : public PixelGreyscaleRow
{
private:
    uint8_t BlackColorTreshold;
    unsigned int ScreenCenter_x;
    unsigned int LaneWidth;
    unsigned int LaneWidthTolerance;
    unsigned int LineWidth;
    PixelRowBlackLine LeftEdge;
    PixelRowBlackLine RightEdge;
    bool isSet_LeftEdge;
    bool isSet_RightEdge;
    bool isSet_Lane;
    

public:
    TrackLane(uint8_t _BlackColorTreshold, unsigned int screenCenter_x, unsigned int lineWidth, unsigned int LaneWidth, unsigned int laneWidthTolerance) : PixelGreyscaleRow() {
        this->clear();
        this->BlackColorTreshold = _BlackColorTreshold;
        this->ScreenCenter_x = screenCenter_x;
        this->LaneWidth = LaneWidth;
        this->LaneWidthTolerance = laneWidthTolerance;
        this->LineWidth = lineWidth;
    }
    ~TrackLane(){

    }

    void setLaneWidth(unsigned int width){
        this->LaneWidth = width;
    }

    PixelRowBlackLine getRightEdge(){
        LaneLines lane;
        if (this->isSet_Lane && this->isSet_RightEdge) {
            return this->RightEdge;
        }

        lane = this->getLaneLines(this->BlackColorTreshold, this->LineWidth, this->LaneWidth, this->LaneWidthTolerance);
        
        this->RightEdge = lane.rightLine;
        this->LeftEdge = lane.leftLine;
        this->isSet_Lane = true;

        if (lineIsValid(this->RightEdge)) {
            this->isSet_RightEdge = true;
        }
        if (lineIsValid(this->LeftEdge)) {
            this->isSet_LeftEdge = true;
        }
        return this->RightEdge;
    }

    PixelRowBlackLine getLeftEdge(){
        LaneLines lane;
        if (this->isSet_Lane && this->isSet_LeftEdge) {
            return this->LeftEdge;
        }

        lane = this->getLaneLines(this->BlackColorTreshold, this->LineWidth, this->LaneWidth, this->LaneWidthTolerance);
        
        this->RightEdge = lane.rightLine;
        this->LeftEdge = lane.leftLine;
        this->isSet_Lane = true;

        if (lineIsValid(this->RightEdge)) {
            this->isSet_RightEdge = true;
        }
        if (lineIsValid(this->LeftEdge)) {
            this->isSet_LeftEdge = true;
        }
        return this->LeftEdge;
    }

    bool isRightEdgeVisible(){
        PixelRowBlackLine rightEdge;

        rightEdge = this->getRightEdge();
        if (rightEdge.beginIndex != rightEdge.endIndex){
            return true;
        }
        else{
            return false;
        }
    }

    bool isLeftEdgeVisible(){
        PixelRowBlackLine leftEdge;
        
        leftEdge = this->getLeftEdge();
        if (leftEdge.beginIndex != leftEdge.endIndex){
            return true;
        }
        else{
            return false;
        }
    }

    int getLaneCenter(){
        int laneCenter = 0;
        PixelRowBlackLine rightEdge;
        PixelRowBlackLine leftEdge;

        leftEdge = this->getLeftEdge();
        rightEdge = this->getRightEdge();

        if (this->isLeftEdgeVisible()) {
            if (this->isRightEdgeVisible()) {
                laneCenter = leftEdge.endIndex + (((int)(rightEdge.beginIndex) - (int)(leftEdge.endIndex)) / 2);
            }
            else{
                laneCenter = (int)leftEdge.endIndex + (int)(((float)this->LaneWidth) / (float)2);
            }
        }
        else{
            if (this->isRightEdgeVisible()) {
                laneCenter = (int)rightEdge.beginIndex - (int)(((float)this->LaneWidth) / (float)2);
            }
            else{
                laneCenter = ScreenCenter_x;
            }
        }
        
        return laneCenter;
    }

    unsigned int autocalibrateLaneWidth(){
        PixelRowBlackLine rightEdge;
        PixelRowBlackLine leftEdge;

        leftEdge = this->getFirstBlackLine(this->BlackColorTreshold, this->LineWidth);
        rightEdge = this->getFirstBlackLine(this->BlackColorTreshold, this->LineWidth);

        if (lineIsValid(leftEdge) && lineIsValid(rightEdge)) {
            this->LaneWidth = (int)(leftEdge.endIndex) - (int)(rightEdge.beginIndex);
            return this->LaneWidth;
        }
        else{
            return 0;
        }
    }

    void clear(){
        PixelGreyscaleRow::clear();
        isSet_LeftEdge = false;
        isSet_RightEdge = false;
        isSet_Lane = false;
    }





    
};


#endif