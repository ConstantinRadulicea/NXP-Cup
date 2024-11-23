#include "Config.h"

void setup(){
  //Serial.begin(SERIAL_PORT_BAUD_RATE);
  LcdMenuSetup(MENU_LEFT_ARROW_BUTTON_PIN, MENU_RIGHT_ARROW_BUTTON_PIN, MENU_INCREMENT_BUTTON_PIN, MENU_DECREMENT_BUTTON_PIN);
}

void loop(){
  settingsMenuRoutine();
    displayParameterValue(String("ENABLE_ENGINE"), String("Disabled"));
  delay(100);
}