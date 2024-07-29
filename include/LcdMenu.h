#ifndef __LCDMENU_H__
#define __LCDMENU_H__

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "GlobalVariables.h"

extern LiquidCrystal_I2C lcd;

void LcdMenuSetup(int left_arrow, int right_arrow, int up_arrow, int down_arrow);
void settingsMenuRoutine();

#endif