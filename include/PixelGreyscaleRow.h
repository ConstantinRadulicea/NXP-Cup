//transform a row of pixels in grey format + fiding lines, construct lines + SMA filter

#ifndef _PIXELGREYSCALEROW_H_
#define _PIXELGREYSCALEROW_H_

#include <stdint.h>
#include <math.h>
#include <vector>

typedef struct PixelRowBlackLine {
    unsigned int beginIndex;
    unsigned int endIndex;  // last pixel index + 1
    int colorTreshold;
}PixelRowBlackLine;

typedef struct LaneLines {
    PixelRowBlackLine leftLine;
    PixelRowBlackLine rightLine;
    unsigned int laneWidth; // rightLine.beginIndex - leftLine.endIndex
}LaneLines;

/*  ToDo:
 *  Add maxPixelLineWidth functionality
 *
 * 
 * 
*/

class PixelGreyscaleRow
{
private:
    std::vector<uint8_t> PixelLine;
    std::vector<PixelRowBlackLine> LinesFound;
    bool allLinesFound = false;
    uint8_t thresholdFoundLines; // contains the value of the pixel which is found black 

public:
    PixelGreyscaleRow(){
        this->clear();
    }
    ~PixelGreyscaleRow(){
    }

    std::vector<uint8_t>& getRow(){
        return PixelLine;
    }

    // NTSC formula: 0.299 ∙ Red + 0.587 ∙ Green + 0.114 ∙ Blue
    static uint8_t RGBtoGreyscale(uint8_t red, uint8_t green, uint8_t blue){// convert RGB to grey scale with the NTSC formula which assigns different weights to the RGB components
    return (uint8_t)(0.299 * red +  0.587 * green + 0.114 * blue);
    }

    void addPixelRGB(uint8_t red, uint8_t green, uint8_t blue){// add the converted pixels to the row
        this->addPixelGreyscale(this->RGBtoGreyscale(red, green, blue));
    }

    void addPixelGreyscale(uint8_t greyscale){
        this->allLinesFound = false;
        this->PixelLine.push_back(greyscale);
    }

    PixelRowBlackLine getNextBlackLine(unsigned int pixelIndex, uint8_t treshold){// finding the next black line, treshold = used to determine when a pixel is considered black
        PixelRowBlackLine line;
        unsigned int linePixels = 0;
        unsigned int i = 0;

        line.beginIndex = 0;
        line.endIndex = 0;
        line.colorTreshold = (int)treshold;

        for (i = pixelIndex; i < this->PixelLine.size(); i++)
        {
            if (this->PixelLine[i] <= treshold) {
                if(linePixels == 0){
                    line.beginIndex = i;
                }
                linePixels++;
            }
            else if(linePixels > 0){
                line.endIndex = i;
                return line;
            }
            else{
                linePixels = 0;
                line.beginIndex = 0;
            }
        }
        if(linePixels > 0){
                line.endIndex = i;
        }
        else{
            line.beginIndex = 0;
        }
        return line;
    }

    unsigned int findAllBlackLines(uint8_t treshold){ 
        PixelRowBlackLine line;
        unsigned int nextPixelIndex = 0;

        if (this->allLinesFound && this->thresholdFoundLines == treshold) {
            return this->LinesFound.size();
        }

        this->LinesFound.clear();
        this->allLinesFound = false;
        this->thresholdFoundLines = -1;
        
        while (nextPixelIndex < this->size())
        {
            line = this->getNextBlackLine(nextPixelIndex, treshold);
            if (! lineIsValid(line)) {
                break;
            }
            this->LinesFound.push_back(line);
            nextPixelIndex = line.endIndex;
        }
        this->thresholdFoundLines = treshold;
        this->allLinesFound = true;

        return this->LinesFound.size();
    }

    std::vector<PixelRowBlackLine> getAllBlackLines(uint8_t treshold){
        this->findAllBlackLines(treshold);
        return this->LinesFound;
    }

    std::vector<PixelRowBlackLine> getAllBlackLines(uint8_t treshold, unsigned int linePixelsMinlen){
        std::vector<PixelRowBlackLine> blackLines;
        this->getAllBlackLines(treshold);
        for(auto iter = this->LinesFound.begin(); iter < this->LinesFound.end(); iter++)
        {
            if (lineWidth(*iter) >= linePixelsMinlen) {
                blackLines.push_back(*iter);
            }
        }
        return blackLines;
    }


    static bool lineIsValid(PixelRowBlackLine& line){
        if (line.beginIndex == line.endIndex) return false;
        else                                  return true;
    }

    static bool laneIsValid(LaneLines& lane){
        if (lineIsValid(lane.leftLine)) return true;
        if (lineIsValid(lane.rightLine)) return true;
        return false;
    }

    static unsigned int lineWidth(PixelRowBlackLine& line){
        return abs((int)line.endIndex - (int)line.beginIndex);
    }

    // positive if right of absolute center, negative if left of absolute center
    int lineDistanceFromAbsoluteCenter(PixelRowBlackLine& line){
        unsigned int middlePixelRow = (unsigned int)((float)this->size() / (float)2);
        return (int)lineAbsoluteCenter(line) - (int)middlePixelRow;
    }

    // returns the center of the line regarding the whole image
    static unsigned int lineAbsoluteCenter(PixelRowBlackLine& line){
        return (int)line.beginIndex + ((float)lineWidth(line) / (float)2);
    }

    // returns the center of the lane regarding the whole image
    static unsigned int laneAbsoluteCenter(PixelRowBlackLine leftLine, PixelRowBlackLine rightLine){
        PixelRowBlackLine line;
        if (leftLine.beginIndex > rightLine.beginIndex)
        {
            line = leftLine;
            leftLine = rightLine;
            rightLine = line;
        }
        return (int)leftLine.endIndex + ((float)laneWidth(leftLine, rightLine) / (float)2);
    }
    
    static unsigned int laneWidth(LaneLines lane){
        PixelRowBlackLine line;
        if (lane.leftLine.beginIndex > lane.rightLine.beginIndex)
        {
            line = lane.leftLine;
            lane.leftLine = lane.rightLine;
            lane.rightLine = line;
        }
        
        return abs((int)lane.rightLine.beginIndex - (int)lane.leftLine.endIndex);
    }

    static unsigned int laneWidth(PixelRowBlackLine leftLine, PixelRowBlackLine rightLine){
        PixelRowBlackLine line;
        if (leftLine.beginIndex > rightLine.beginIndex)
        {
            line = leftLine;
            leftLine = rightLine;
            rightLine = line;
        }
        
        return abs((int)rightLine.beginIndex - (int)leftLine.endIndex);
    }

    PixelRowBlackLine getFirstBlackLine_old(uint8_t treshold, unsigned int linePixelsMinlen){
        PixelRowBlackLine line;
        unsigned int nextPixelIndex = 0;

        if (linePixelsMinlen == 0) {
            line.beginIndex = 0;
            line.endIndex = 0;
            line.colorTreshold = -1;
            return line;
        }
        
        while (nextPixelIndex < this->size())
        {
            line = this->getNextBlackLine(nextPixelIndex, treshold);
            if ((line.endIndex - line.beginIndex) >= linePixelsMinlen) {
                return line;
            }
            else if (line.beginIndex == line.endIndex){     // no line found with this tresh hold
                return line;
            }
            else{
                nextPixelIndex = line.endIndex+1;
            }
        }

        line.beginIndex = 0;
        line.endIndex = 0;
        return line;
    }

    // get the first left black line
    PixelRowBlackLine getFirstBlackLine(uint8_t treshold, unsigned int linePixelsMinlen){
        PixelRowBlackLine line;
        std::vector<PixelRowBlackLine> lines = this->getAllBlackLines(treshold);
        for (int i = 0; i < (int)lines.size(); i++)
        {
            if (lineWidth(lines[i]) >= linePixelsMinlen){
                return lines[i];
            }
        }
        memset(&line, 0, sizeof(PixelRowBlackLine));
        line.colorTreshold = -1;
        return line;
    }

    PixelRowBlackLine getLastBlackLine_old(uint8_t treshold, unsigned int linePixelsMinlen){
        PixelRowBlackLine lastValidLine;
        PixelRowBlackLine line;
        unsigned int nextPixelIndex = 0;

        lastValidLine.beginIndex = 0;
        lastValidLine.endIndex = 0;
        lastValidLine.colorTreshold = -1;

        if (linePixelsMinlen == 0) {
            return lastValidLine;
        }
        

        while (nextPixelIndex < this->size())
        {
            line = this->getNextBlackLine(nextPixelIndex, treshold);
            if ((line.endIndex - line.beginIndex) >= linePixelsMinlen) {    // found a good candidate line
                lastValidLine = line;
                nextPixelIndex = lastValidLine.endIndex+1;
            }
            else if (line.beginIndex == line.endIndex){     // no line found with this tresh hold
                return lastValidLine;
            }
            else{
                nextPixelIndex = line.endIndex+1;           // found a bad candidate line
            }
        }
        return lastValidLine;
    }

    // get the further right black line
    PixelRowBlackLine getLastBlackLine(uint8_t treshold, unsigned int linePixelsMinlen){
        PixelRowBlackLine line;
        std::vector<PixelRowBlackLine> lines = this->getAllBlackLines(treshold);
        for (int i = lines.size()-1; i >= 0; i--)
        {
            if (lineWidth(lines[i]) >= linePixelsMinlen){
                return lines[i];
            }
        }
        memset(&line, 0, sizeof(PixelRowBlackLine));
        line.colorTreshold = -1;
        return line;
    }

    // builds a lane using only one line, the other line will be an invalid one.
    // this function is used to determine if this line is the rightLine of the leftLine
    LaneLines buildLaneFromLine(PixelRowBlackLine line, unsigned int laneWidth_){
        LaneLines laneLines;
        unsigned int middlePixelRow = (unsigned int)((float)this->size() / (float)2);
        memset(&laneLines, 0, sizeof(LaneLines));
        if (lineIsValid(line))
        {
            if (lineAbsoluteCenter(line) > middlePixelRow) {
                laneLines.rightLine = line;
            }else{
                laneLines.leftLine = line;
            }
            laneLines.laneWidth = 0;
        }
        return laneLines;
    }

    // builds a lane structure from 2 lines regarding lane width.
    // if lane width condition is not met, the lane will contein only one line
    LaneLines buildLaneFromLines(PixelRowBlackLine leftLine, PixelRowBlackLine rightLine, unsigned int laneWidth_, unsigned int laneWidthTolerance){
        PixelRowBlackLine line;
        LaneLines laneLines;
        unsigned int middlePixelRow = (unsigned int)((float)this->size() / (float)2);

        memset(&laneLines, 0, sizeof(LaneLines));

        if (lineIsValid(leftLine) && lineIsValid(rightLine))
        {
            if (leftLine.beginIndex > rightLine.beginIndex) {
                line = leftLine;
                leftLine = rightLine;
                rightLine = line;
            }

            if(abs((int)laneWidth(leftLine, rightLine) - (int) laneWidth_) <= (int)laneWidthTolerance){  // the width of the line fits our criteria
                laneLines.leftLine = leftLine;
                laneLines.rightLine = rightLine;
                laneLines.laneWidth = laneWidth(leftLine, rightLine);
            }else{
                if(leftLine.endIndex <= middlePixelRow && rightLine.beginIndex >= middlePixelRow){   // the lines doesent fits our criteria, but center of image is between lane lines
                    if(laneAbsoluteCenter(leftLine, rightLine) >= middlePixelRow){
                        laneLines.leftLine = leftLine;
                    }
                    else{
                        laneLines.rightLine = rightLine;
                    }
                }
                else{
                    if(leftLine.endIndex > middlePixelRow){
                        laneLines.rightLine = leftLine;
                    }
                    else{
                        laneLines.leftLine = rightLine;
                    }
                }
                laneLines.laneWidth = 0;
            }
        }
        else if(lineIsValid(leftLine)){
            laneLines = buildLaneFromLine(leftLine, laneWidth_);
        }
        else if(lineIsValid(rightLine)){
            laneLines = buildLaneFromLine(rightLine, laneWidth_);
        }
        return laneLines;
    }
    // returns the best 2 lane lines regardig lanewidth and line width
    LaneLines getLaneLines(uint8_t treshold, unsigned int linePixelsMinWidth, unsigned int laneWidth_, unsigned int laneWidthTolerance){
        LaneLines laneLines, tempLaneLines;
        std::vector<PixelRowBlackLine> lines = this->getAllBlackLines(treshold, linePixelsMinWidth);

        memset(&laneLines, 0, sizeof(LaneLines));

        if (lines.size() == 1) {
            laneLines = this->buildLaneFromLine(lines[0], laneWidth_);
            return laneLines;
        }
        else if(lines.size() == 2){
            laneLines = this->buildLaneFromLines(lines[0], lines[1], laneWidth_, laneWidthTolerance);
            return laneLines;
        }
        

        for (int i = 0; i < (int)(lines.size()); i++)
        {
            for (int j = i+1; j < (int)lines.size(); j++)
            {
                tempLaneLines = this->buildLaneFromLines(lines[i], lines[j], laneWidth_, laneWidthTolerance);
                
                if(! laneIsValid(laneLines)) {
                    laneLines = tempLaneLines;
                }
                else if (abs((int)laneLines.laneWidth - (int)laneWidth_) > abs((int)tempLaneLines.laneWidth - (int)laneWidth_)) {   // new lane is better than previous
                    laneLines = tempLaneLines;
                }
                else if(laneIsValid(tempLaneLines) && !(lineIsValid(laneLines.leftLine) && lineIsValid(laneLines.rightLine))){
                    if (lineIsValid(laneLines.leftLine))
                    {
                        if (abs(lineDistanceFromAbsoluteCenter(laneLines.leftLine)) > abs(lineDistanceFromAbsoluteCenter(tempLaneLines.leftLine)) || abs(lineDistanceFromAbsoluteCenter(laneLines.leftLine)) > abs(lineDistanceFromAbsoluteCenter(tempLaneLines.rightLine)))
                        {
                            laneLines = tempLaneLines;
                        }
                    }
                    else{
                        if (abs(lineDistanceFromAbsoluteCenter(laneLines.rightLine)) > abs(lineDistanceFromAbsoluteCenter(tempLaneLines.leftLine)) || abs(lineDistanceFromAbsoluteCenter(laneLines.rightLine)) > abs(lineDistanceFromAbsoluteCenter(tempLaneLines.rightLine)))
                        {
                            laneLines = tempLaneLines;
                        }
                    }
                }
            }
        }
        return laneLines;
    }


    void clear(){
        this->PixelLine.clear();
        this->allLinesFound = false;
        this->thresholdFoundLines = -1;
        this->LinesFound.clear();
    }

    std::size_t size(){
        return this->PixelLine.size();
    }

    // filter used to eliminate high frequency spikes in pixel color change
    static std::vector<uint8_t> simpleMovingAverage_uint8_t(std::vector<uint8_t>& row, unsigned int meanSamples) {
        std::vector<uint8_t> movingAverage;
        int localSamplesSum = 0;
        unsigned int localSamplesCount = 0;
        float localAverage = 0.0;

        movingAverage.reserve(row.size());


        for (int i = 0; i < (int)row.size(); i++)
        {

            if (i < (int)meanSamples)
            {
                localSamplesCount += 1;
                localSamplesSum += row[i];
            }
            else {
                localSamplesSum += row[i];
                localSamplesSum -= row[i - meanSamples];
            }
            
            if (localSamplesSum == 0) {
                localAverage = 0;
            }
            else{
                localAverage = (float)localSamplesSum / (float)localSamplesCount;
            }
            
            movingAverage.push_back((uint8_t)localAverage);
        }

        return movingAverage;
    }

    void applySmaFilter(unsigned meanSamples = 3){
        std::vector<uint8_t> movingAverage;
        movingAverage = this->simpleMovingAverage_uint8_t(this->PixelLine, meanSamples);
        this->PixelLine = movingAverage;
    }
};


#endif
