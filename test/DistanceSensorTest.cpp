#include "Config.h"

void setup(){
     DistanceSensorsSetupAnalog(
    DISTANCE_SENSOR1_ANALOG_PIN,
    DISTANCE_SENSOR2_ANALOG_PIN,
    DISTANCE_SENSOR3_ANALOG_PIN
    );
   Serial.begin(SERIAL_PORT_BAUD_RATE);
}


void loop(){
  Serial.println(getFrontObstacleDistanceAnalog_m());
  delay(100);
}