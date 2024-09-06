#include "Config.h"

PWMServo Trigger;

void setup(){
  DistanceSensorsSetupAnalog(
    DISTANCE_SENSOR1_ANALOG_PIN,
    DISTANCE_SENSOR2_ANALOG_PIN,
    DISTANCE_SENSOR3_ANALOG_PIN
  );
  pinMode(EDF_MOTOR_PIN, OUTPUT);
   Serial.begin(SERIAL_PORT_BAUD_RATE);
   //Trigger.attach(EDF_MOTOR_PIN,30,97000); // (pin, min pulse width, max pulse width in microseconds) 
}


void loop(){
  Serial.println(getFrontObstacleDistanceAnalog_m());
  delay(10);
}