//transform a row of pixels in grey format + fiding lines, construct lines + SMA filter

#ifndef _PIXELGREYSCALEROW_H_
#define _PIXELGREYSCALEROW_H_

#include <stdint.h>
#include <math.h>
#include <vector>


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

typedef struct RGBcolor {
	uint8_t R;
	uint8_t G;
	uint8_t B;
}RGBcolor;

typedef struct HSVcolor {
	float H;
	float S;
	float V;
}HSVcolor;

typedef struct PixelRowBlackLine { // black line
    unsigned int beginIndex;
    unsigned int endIndex;  // last pixel index + 1
    float colorTreshold;
}PixelRowBlackLine;

typedef struct LaneLines { // lane = both left and right line
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
    std::vector<float> PixelLine;// contains the row of pixels (greyscaled)
    std::vector<PixelRowBlackLine> LinesFound;
    bool allLinesFound = false;
    float thresholdFoundLines;

public:

    void clear(){
        this->PixelLine.clear(); //vector method
        this->allLinesFound = false;
        this->thresholdFoundLines = -1;
        this->LinesFound.clear(); //vector method
    }

    PixelGreyscaleRow(){ //init constructor
        this->clear();   // class method != vector method
    }
    ~PixelGreyscaleRow(){
    }

    std::vector<float>& getRow(){
        return PixelLine;
    }

    // NTSC formula: 0.299 ∙ Red + 0.587 ∙ Green + 0.114 ∙ Blue
    static float RGBtoGreyscale(uint8_t red, uint8_t green, uint8_t blue){// convert RGB to grey scale with the NTSC formula which assigns different weights to the RGB components
    return (float)(0.299f * (float)red +  0.587f * (float)green + 0.114f * (float)blue);
    }


    static HSVcolor rgb2hsv(RGBcolor rgbColor) {
	//R, G and B input range = 0 � 255
	//H, S and V output range = 0 � 1.0
	HSVcolor hsvColor;
	float del_R, del_G, del_B;

	float var_R = ((float)rgbColor.R / 255.0f);
	float var_G = ((float)rgbColor.G / 255.0f);
	float var_B = ((float)rgbColor.B / 255.0f);

	float var_Min = MIN(MIN(var_R, var_G), var_B);    //Min. value of RGB
	float var_Max = MAX(MAX(var_R, var_G), var_B);    //Max. value of RGB
	float del_Max = var_Max - var_Min;             //Delta RGB value

/*
    Serial.println(rgbColor.R);
    Serial.println(rgbColor.G);
    Serial.println(rgbColor.B);
    Serial.println(var_R);
    Serial.println(var_G);
    Serial.println(var_B);
    Serial.println();
    */

	hsvColor.V = var_Max;
    //Serial.println(hsvColor.V);

		if (del_Max == 0.0f)                     //This is a gray, no chroma...
		{
			hsvColor.H = 0.0f;
			hsvColor.S = 0.0f;
		}
		else                                    //Chromatic data...
		{
			hsvColor.S = del_Max / var_Max;

			del_R = (((var_Max - var_R) / 6.0f) + (del_Max / 2.0f)) / del_Max;
			del_G = (((var_Max - var_G) / 6.0f) + (del_Max / 2.0f)) / del_Max;
			del_B = (((var_Max - var_B) / 6.0f) + (del_Max / 2.0f)) / del_Max;

			if (var_R == var_Max) hsvColor.H = del_B - del_G;
			else if (var_G == var_Max) hsvColor.H = (1.0f / 3.0f) + del_R - del_B;
			else if (var_B == var_Max) hsvColor.H = (2.0f / 3.0f) + del_G - del_R;

			if (hsvColor.H < 0.0f) hsvColor.H += 1.0f;
			if (hsvColor.H > 1.0f) hsvColor.H -= 1.0f;
		}
        
		return hsvColor;
}


    void addPixelGreyscale(float greyscale){
        this->allLinesFound = false;
        this->PixelLine.push_back(greyscale);
    }


    void addPixelRGB(uint8_t red, uint8_t green, uint8_t blue){// add the converted pixels to the row
        this->addPixelGreyscale(this->RGBtoGreyscale(red, green, blue));
    }


    PixelRowBlackLine getNextBlackLine(unsigned int pixelIndex, float treshold){// finding the next black line, treshold = used to determine when a pixel is considered black
        PixelRowBlackLine line; //line instance
        unsigned int linePixels = 0; //black line pixels
        unsigned int i = 0;

        //init
        line.beginIndex = 0;
        line.endIndex = 0;
        line.colorTreshold = (float)treshold;

        for (i = pixelIndex; i < this->PixelLine.size(); i++) 
        {
            if (this->PixelLine[i] <= treshold) { // treshold: 0-255 where 0 is black, 255 white; '<= treshold' is a black pixel
                if(linePixels == 0){              // first condition to be approved else we keep going through the vector
                    line.beginIndex = i; //first index found
                }
                linePixels++; //black line pixels
            }
            else if(linePixels > 0){ 
                line.endIndex = i; //last index found
                return line;
            }
            else{ // needed if there are more lines (which is the case; 1 lane contains 2 lines)
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


    std::size_t size(){
        return this->PixelLine.size();
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


    unsigned int findAllBlackLines(float treshold){ 
        PixelRowBlackLine line; // line instance
        unsigned int nextPixelIndex = 0;

        if (this->allLinesFound && this->thresholdFoundLines >= treshold) {
            return this->LinesFound.size();
        }
        
        //init
        this->LinesFound.clear(); //vector method
        this->allLinesFound = false;
        this->thresholdFoundLines = -1;
        
        while (nextPixelIndex < this->size()) // PixelLine.size()
        {
            line = this->getNextBlackLine(nextPixelIndex, treshold);
            if (! lineIsValid(line)) {
                break;
            }

            this->LinesFound.push_back(line); // vector for the lines which are found
            nextPixelIndex = line.endIndex;
        }

        this->thresholdFoundLines = treshold;
        this->allLinesFound = true;

        return this->LinesFound.size(); // number of lines
    }


    std::vector<PixelRowBlackLine> getAllBlackLines(float treshold){
        this->findAllBlackLines(treshold);
        return this->LinesFound;
    }


    std::vector<PixelRowBlackLine> getAllBlackLines(float treshold, unsigned int linePixelsMinlen){ //linePixelsMinlen = min pixels for 1 black line
        std::vector<PixelRowBlackLine> blackLines;
        this->getAllBlackLines(treshold);

        for(auto iter = this->LinesFound.begin(); iter < this->LinesFound.end(); iter++) // going through the linesFound vector
        {
            if (lineWidth(*iter) >= linePixelsMinlen) {
                blackLines.push_back(*iter);
            }
        }
        return blackLines;
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


    // positive if right of absolute center, negative if left of absolute center
    int lineDistanceFromAbsoluteCenter(PixelRowBlackLine& line){
        unsigned int middlePixelRow = (unsigned int)((float)this->size() / (float)2);
        return (int)lineAbsoluteCenter(line) - (int)middlePixelRow;
    }


    static unsigned int lineWidth(PixelRowBlackLine& line){
        return abs((int)line.endIndex - (int)line.beginIndex); // line width (number of pixels)
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


    PixelRowBlackLine getFirstBlackLine_old(float treshold, unsigned int linePixelsMinlen){
        PixelRowBlackLine line;

        unsigned int nextPixelIndex = 0;

        if (linePixelsMinlen == 0) {
            line.beginIndex = 0;
            line.endIndex = 0;
            line.colorTreshold = -1.0f;
            return line;
        }
        
        while (nextPixelIndex < this->size()) // PixelLine.size()
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
    PixelRowBlackLine getFirstBlackLine(float treshold, unsigned int linePixelsMinlen){
        PixelRowBlackLine line;
        std::vector<PixelRowBlackLine> lines = this->getAllBlackLines(treshold);
        for (int i = 0; i < (int)lines.size(); i++)
        {
            if (lineWidth(lines[i]) >= linePixelsMinlen){
                return lines[i];
            }
        }

        memset(&line, 0, sizeof(PixelRowBlackLine)); // set all bytes of &line to 0.
        line.colorTreshold = -1.0f;
        return line;
    }



    PixelRowBlackLine getLastBlackLine_old(float treshold, unsigned int linePixelsMinlen){
        PixelRowBlackLine lastValidLine;
        PixelRowBlackLine line;
        unsigned int nextPixelIndex = 0;

        lastValidLine.beginIndex = 0;
        lastValidLine.endIndex = 0;
        lastValidLine.colorTreshold = -1.0f;

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
    PixelRowBlackLine getLastBlackLine(float treshold, unsigned int linePixelsMinlen){
        PixelRowBlackLine line;
        std::vector<PixelRowBlackLine> lines = this->getAllBlackLines(treshold);
        for (int i = lines.size()-1; i >= 0; i--)
        {
            if (lineWidth(lines[i]) >= linePixelsMinlen){
                return lines[i];
            }
        }

        memset(&line, 0, sizeof(PixelRowBlackLine));
        line.colorTreshold = -1.0f;
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
    // returns the best 2 lane lines regardig lane width and line width
    // If multiple lines are detected, it will choose the lines which forms a lane of the closest width laneWidth_ with an error of laneWidthTolerance.
    // If none of the lines can form a lane regarding the parameters laneWidth_ and laneWidthTolerance, the line closest to the center will be chosen 
    // and whether this line is on the right or on the left from the center, it will be the leftLine or rightLine
    LaneLines getLaneLines(float treshold, unsigned int linePixelsMinWidth, unsigned int laneWidth_, unsigned int laneWidthTolerance){
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


    // filter used to eliminate high frequency spikes in pixel color change
    static std::vector<float> simpleMovingAverage_uint8_t(std::vector<float>& row, unsigned int meanSamples) {
        std::vector<float> movingAverage;
        float localSamplesSum = 0.0f;
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
            
            if (localSamplesSum <= 0.001) {
                localAverage = 00.f;
            }
            else{
                localAverage = (float)localSamplesSum / (float)localSamplesCount;
            }
            
            movingAverage.push_back((float)localAverage);
        }

        return movingAverage;
    }

    void applySmaFilter(unsigned meanSamples = 3){
        std::vector<float> movingAverage;
        movingAverage = this->simpleMovingAverage_uint8_t(this->PixelLine, meanSamples);
        this->PixelLine = movingAverage;
    }
};


#endif
