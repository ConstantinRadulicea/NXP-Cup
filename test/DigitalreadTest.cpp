#include <Arduino.h>
#include "Config.h"


void setup(){
    pinMode(20, INPUT_PULLDOWN);
    pinMode(21, INPUT_PULLUP);
    pinMode(22, INPUT_PULLUP);
    Serial.begin(230400);
    while (!Serial) {delay(100); }
    
}

void loop(){
    Serial.print(digitalRead(20));
    Serial.print("\t");
    Serial.print(digitalRead(21));
    Serial.print("\t");
    Serial.print(digitalRead(22));
    Serial.println();
    delay(100);
}