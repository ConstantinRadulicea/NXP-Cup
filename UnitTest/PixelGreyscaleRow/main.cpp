#include "../../include/PixelGreyscaleRow.h"








int main() {
	PixelGreyscaleRow pixelGreyscaleRow;
	LaneLines lane;


	pixelGreyscaleRow.addPixelGreyscale(50);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(50);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(50);
	
	lane = pixelGreyscaleRow.getLaneLines(180, 1, 5, 1);	// result 5-6		11-12


	pixelGreyscaleRow.clear();
	pixelGreyscaleRow.addPixelGreyscale(50);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(50);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(50);

	lane = pixelGreyscaleRow.getLaneLines(180, 1, 5, 1);	// result 4-5		10-11


	pixelGreyscaleRow.clear();
	pixelGreyscaleRow.addPixelGreyscale(50);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(50);
	pixelGreyscaleRow.addPixelGreyscale(50);
	pixelGreyscaleRow.addPixelGreyscale(50);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(50);

	lane = pixelGreyscaleRow.getLaneLines(180, 1, 5, 1);	// result 2-5		0-0




	pixelGreyscaleRow.clear();
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(50);

	lane = pixelGreyscaleRow.getLaneLines(180, 1, 5, 1);	// result 0-0		3-4




	pixelGreyscaleRow.clear();
	pixelGreyscaleRow.addPixelGreyscale(50);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(220);
	pixelGreyscaleRow.addPixelGreyscale(50);
	pixelGreyscaleRow.addPixelGreyscale(50);
	pixelGreyscaleRow.addPixelGreyscale(50);

	lane = pixelGreyscaleRow.getLaneLines(180, 1, 5, 1);	// result 0-0		3-6
    
	return 0;
}

