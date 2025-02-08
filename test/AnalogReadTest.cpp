#include <Arduino.h>
#include "Config.h"


void setup(){
    pinMode(DISTANCE_SENSOR1_ANALOG_PIN, INPUT);
    pinMode(DISTANCE_SENSOR2_ANALOG_PIN, INPUT);
    pinMode(DISTANCE_SENSOR3_ANALOG_PIN, INPUT);
    Serial.begin(230400);
    while (!Serial) {delay(100); }
    
}

void loop(){
    Serial.print(analogRead(DISTANCE_SENSOR1_ANALOG_PIN));
    Serial.print("\t");
    Serial.print(analogRead(DISTANCE_SENSOR2_ANALOG_PIN));
    Serial.print("\t");
    Serial.print(analogRead(DISTANCE_SENSOR3_ANALOG_PIN));
    Serial.println();
    delay(100);
}