#ifndef _TRACKLANE_H_
#define _TRACKLANE_H_

#include "PixelGreyscaleRow.h"

class TrackLane : public PixelGreyscaleRow
{
private:
    uint8_t BlackColorTreshold;
    unsigned int ScreenCenter_x;
    unsigned int LaneWidth;
    PixelRowBlackLine LeftEdge;
    PixelRowBlackLine RightEdge;
    bool isSet_LeftEdge;
    bool isSet_RightEdge;

public:
    TrackLane(uint8_t _BlackColorTreshold, unsigned int screenCenter_x, unsigned int LaneWidth) : PixelGreyscaleRow() {
        this->clear();
        this->BlackColorTreshold = _BlackColorTreshold;
        this->ScreenCenter_x = screenCenter_x;
        this->LaneWidth = LaneWidth;
    }
    ~TrackLane(){

    }

    PixelRowBlackLine getRightEdge(){
        if (isSet_RightEdge) {
            return this->RightEdge;
        }

        PixelRowBlackLine rightEdge;
        PixelRowBlackLine leftEdge;
        
        leftEdge = this->getFirstBlackLine(this->BlackColorTreshold, 2);
        rightEdge = this->getLastBlackLine(this->BlackColorTreshold, 2);

        if (rightEdge.beginIndex > this->ScreenCenter_x) {
            this->RightEdge = rightEdge;
            this->isSet_RightEdge = true;
            return rightEdge;
        }
        else if (leftEdge.beginIndex > this->ScreenCenter_x){
            this->RightEdge = leftEdge;
            this->isSet_RightEdge = true;
            return leftEdge;
        }
        else{
            leftEdge.beginIndex = 0;
            leftEdge.endIndex = 0;
            this->RightEdge = leftEdge;
            this->isSet_RightEdge = true;
            return leftEdge;
        }
    }

    PixelRowBlackLine getLeftEdge(){
        if (isSet_LeftEdge) {
            return this->LeftEdge;
        }
        
        PixelRowBlackLine rightEdge;
        PixelRowBlackLine leftEdge;
        
        leftEdge = this->getFirstBlackLine(this->BlackColorTreshold, 2);
        rightEdge = this->getLastBlackLine(this->BlackColorTreshold, 2);

        if (leftEdge.beginIndex <= this->ScreenCenter_x) {
            this->LeftEdge = leftEdge;
            this->isSet_LeftEdge = true;
            return leftEdge;
        }
        else if (rightEdge.beginIndex <= this->ScreenCenter_x){
            this->LeftEdge = rightEdge;
            this->isSet_LeftEdge = true;
            return rightEdge;
        }
        else{
            leftEdge.beginIndex = 0;
            leftEdge.endIndex = 0;
            this->LeftEdge = leftEdge;
            this->isSet_LeftEdge = true;
            return leftEdge;
        }
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
                laneCenter = (int)leftEdge.endIndex + (int)(((float)this->LaneWidth) / 2);
            }
        }
        else{
            if (this->isRightEdgeVisible()) {
                laneCenter = (int)rightEdge.beginIndex - (int)(((float)this->LaneWidth) / 2);
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

        leftEdge = this->getLeftEdge();
        rightEdge = this->getRightEdge();

        if (this->isLeftEdgeVisible() && this->isRightEdgeVisible())
        {
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
    }





    
};


#endif