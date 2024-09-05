#include "Config.h"

void setup(){
  LcdMenuSetup(MENU_LEFT_ARROW_BUTTON_PIN, MENU_RIGHT_ARROW_BUTTON_PIN, MENU_INCREMENT_BUTTON_PIN, MENU_DECREMENT_BUTTON_PIN);
}

void loop(){
  settingsMenuRoutine();
  delay(50);
}