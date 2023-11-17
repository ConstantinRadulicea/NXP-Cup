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

typedef struct laneLines {
    PixelRowBlackLine leftLine;
    PixelRowBlackLine rightLine;
}laneLines;

class PixelGreyscaleRow
{
private:
    std::vector<uint8_t> PixelLine;
    std::vector<PixelRowBlackLine> LinesFound;
    bool allLinesFound = false;
    uint8_t thresholdFoundLines;

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
    static uint8_t RGBtoGreyscale(uint8_t red, uint8_t green, uint8_t blue){
    return (uint8_t)(0.299 * red +  0.587 * green + 0.114 * blue);
    }

    void addPixelRGB(uint8_t red, uint8_t green, uint8_t blue){
        this->addPixelGreyscale(this->RGBtoGreyscale(red, green, blue));
    }

    void addPixelGreyscale(uint8_t greyscale){
        this->allLinesFound = false;
        this->PixelLine.push_back(greyscale);
    }

    PixelRowBlackLine getNextBlackLine(unsigned int pixelIndex, uint8_t treshold){
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

    std::vector<PixelRowBlackLine>& getAllBlackLines(uint8_t treshold){
        PixelRowBlackLine line;
        unsigned int nextPixelIndex = 0;

        if (this->allLinesFound && this->thresholdFoundLines == treshold) {
            return this->LinesFound;
        }

        this->allLinesFound = false;
        
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

        return this->LinesFound;
    }

    static bool lineIsValid(PixelRowBlackLine& line){
        if (line.beginIndex == line.endIndex) return false;
        else                                  return true;
    }
    
    static unsigned int lineWidth(PixelRowBlackLine& line){
        return line.endIndex - line.beginIndex;
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

    PixelRowBlackLine getFirstBlackLine(uint8_t treshold, unsigned int linePixelsMinlen){
        std::vector<PixelRowBlackLine>& lines = this->getAllBlackLines(treshold);
        for (int i = 0; i < lines.size(); i++)
        {
            if (lineWidth(lines[i]) >= linePixelsMinlen){
                return lines[i];
            }
        }
        return PixelRowBlackLine{.beginIndex = 0, .endIndex = 0, .colorTreshold = -1};
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

    PixelRowBlackLine getLastBlackLine(uint8_t treshold, unsigned int linePixelsMinlen){
        std::vector<PixelRowBlackLine>& lines = this->getAllBlackLines(treshold);
        for (int i = lines.size()-1; i >= 0; i--)
        {
            if (lineWidth(lines[i]) >= linePixelsMinlen){
                return lines[i];
            }
        }
        return PixelRowBlackLine{.beginIndex = 0, .endIndex = 0, .colorTreshold = -1};
    }


    laneLines getLaneLines(uint8_t treshold, unsigned int linePixelsMinWidth){

    }


    void clear(){
        this->PixelLine.clear();
        this->allLinesFound = false;
        this->LinesFound.clear();
    }

    std::size_t size(){
        return this->PixelLine.size();
    }

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