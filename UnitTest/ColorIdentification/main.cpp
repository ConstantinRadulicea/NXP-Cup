#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include <cstdint>

// https://colorizer.org/

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

typedef struct RGBcolor {
	uint8_t R;
	uint8_t G;
	uint8_t B;
}RGBcolor;


typedef struct CIELABcolor {
	float L;
	float a;
	float b;
}CIELABcolor;

typedef struct HSVcolor {
	float H;
	float S;
	float V;
}HSVcolor;


float rgb2lab_normalizeRgbChannel(uint8_t rgbChannel) {
	float channel = ((float)rgbChannel) / 255.0;
	if (channel > 0.04045)
	{
		channel = powf(((channel + 0.055) / 1.055), 2.4);
	}
	else {
		channel = channel / 12.92;
	}

	channel = channel * 100.0;
	return channel;
}


float rgb2lab_normalizeXyzChannel(float xyzChannel) {
	float channel = xyzChannel;
	if (channel > 0.008856)
	{
		channel = pow(channel, 0.3333);
	}
	else {
		channel = (7.787 * channel) + (16.0 / 116.0);
	}
	return channel;
}


/*
# RGB to Lab conversion

# Step 1: RGB to XYZ
#         http://www.easyrgb.com/index.php?X=MATH&H=02#text2
# Step 2: XYZ to Lab
#         http://www.easyrgb.com/index.php?X=MATH&H=07#text7
*/
CIELABcolor rgb2lab(RGBcolor rgbColor) {
	CIELABcolor labColor;
	float r, g, b;
	float X, Y, Z;

	r = rgb2lab_normalizeRgbChannel(rgbColor.R);
	g = rgb2lab_normalizeRgbChannel(rgbColor.G);
	b = rgb2lab_normalizeRgbChannel(rgbColor.B);

	X = r * 0.4124 + g * 0.3576 + b * 0.1805;
	Y = r * 0.2126 + g * 0.7152 + b * 0.0722;
	Z = r * 0.0193 + g * 0.1192 + b * 0.9505;

	// Observer= 2�, Illuminant= D65
	X = rgb2lab_normalizeXyzChannel(X / 95.0470);	// ref_X = 95.047
	Y = rgb2lab_normalizeXyzChannel(Y / 100.0);		// ref_Y = 100.000
	Z = rgb2lab_normalizeXyzChannel(Z / 108.883);	// ref_Z = 108.883

	labColor.L = (116.0 * Y) - 16.0; // L
	labColor.a = 500.0 * (X - Y);  // a
	labColor.b = 200.0 * (Y - Z);  // b

	return labColor;
}

HSVcolor rgb2hsv(RGBcolor rgbColor) {
	//R, G and B input range = 0 � 255
	//H, S and V output range = 0 � 1.0
	HSVcolor hsvColor;
	float del_R, del_G, del_B;

	float var_R = ((float)rgbColor.R / 255.0);
	float var_G = ((float)rgbColor.G / 255.0);
	float var_B = ((float)rgbColor.B / 255.0);

	float var_Min = MIN(MIN(var_R, var_G), var_B);    //Min. value of RGB
	float var_Max = MAX(MAX(var_R, var_G), var_B);    //Max. value of RGB
	float del_Max = var_Max - var_Min;             //Delta RGB value

	hsvColor.V = var_Max;

		if (del_Max == 0)                     //This is a gray, no chroma...
		{
			hsvColor.H = 0;
			hsvColor.S = 0;
		}
		else                                    //Chromatic data...
		{
			hsvColor.S = del_Max / var_Max;

			del_R = (((var_Max - var_R) / 6.0) + (del_Max / 2)) / del_Max;
			del_G = (((var_Max - var_G) / 6.0) + (del_Max / 2)) / del_Max;
			del_B = (((var_Max - var_B) / 6.0) + (del_Max / 2)) / del_Max;

			if (var_R == var_Max) hsvColor.H = del_B - del_G;
			else if (var_G == var_Max) hsvColor.H = (1.0 / 3.0) + del_R - del_B;
			else if (var_B == var_Max) hsvColor.H = (2.0 / 3.0) + del_G - del_R;

			if (hsvColor.H < 0.0) hsvColor.H += 1.0;
			if (hsvColor.H > 1.0) hsvColor.H -= 1.0;
		}
		return hsvColor;
}

int main() {
	CIELABcolor labColor;
	HSVcolor hsvColor;
	RGBcolor rgbColor = { 25.5, 25.5, 25.5 };
	labColor = rgb2lab(rgbColor);
	hsvColor = rgb2hsv(rgbColor);
	return 0;
}