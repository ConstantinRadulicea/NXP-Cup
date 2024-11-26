/*
* Copyright 2024 Constantin Dumitru Petre RĂDULICEA
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

#ifndef __LCDMENU_H__
#define __LCDMENU_H__

#include <Arduino.h>
#include <Wire.h>

#include "GlobalVariables.h"
#include "FloatToString.h"

#define LCD_LIBRARY_ADAFRUIT 0
#define LCD_LIBRARY_SSD1306Ascii 1



#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#if LCD_LIBRARY_ADAFRUIT != 0
    #include <Adafruit_GFX.h>
    #include <Adafruit_SSD1306.h>

    #define SCREEN_WIDTH 128
    #define SCREEN_HEIGHT 64

    #define PARAMETER_NAME_TEXT_SIZE 1
    #define PARAMETER_VALUE_TEXT_SIZE 1
    #define PARAMETER_NAME_TEXT_COLOR WHITE
    #define PARAMETER_VALUE_TEXT_COLOR WHITE

    extern Adafruit_SSD1306 display;
#endif


#if LCD_LIBRARY_SSD1306Ascii != 0
    #include "SSD1306Ascii.h"
    #include "SSD1306AsciiWire.h"

    // 0X3C+SA0 - 0x3C or 0x3D
    #define I2C_ADDRESS 0x3C

    // Define proper RST_PIN if required.
    #define RST_PIN -1

    extern SSD1306AsciiWire display;
#endif







void displayParameterValue(String parameter, String value);
void LcdMenuSetup(int left_arrow, int right_arrow, int up_arrow, int down_arrow);
void settingsMenuRoutine();

#endif