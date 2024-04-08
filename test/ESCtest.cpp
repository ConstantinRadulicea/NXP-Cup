/*
        Arduino Brushless Motor Control
     by Dejan, https://howtomechatronics.com
*/

#include <Arduino.h>
#include "Config.h"
#include <PWMServo.h>

PWMServo ESC;     // create servo object to control the ESC

int potValue = 83;  // value from the analog pin
float time1;
#define DRIVER_MOTOR_PIN  9

void setup() {

    ESC.attach(DRIVER_MOTOR_PIN,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 

  time1 = (float)millis();
  while ((float)millis() - time1 < 10000)
  {
    ESC.write(potValue);    // Send the signal to the ESC
  }

  Serial.begin(115200);
  while (!Serial)
  {
    delay(100);
  }
  
  // Attach the ESC on pin 9

}

void loop() {
    if (potValue > 180)
    {
        potValue = 0;
    }
    Serial.println(potValue);
  time1 = (float)millis();
  while ((float)millis() - time1 < 200)
  {
    ESC.write(potValue);    // Send the signal to the ESC
    delay(4);
  }
  potValue += 1;
}