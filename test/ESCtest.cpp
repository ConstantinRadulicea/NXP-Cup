/*
        Arduino Brushless Motor Control
     by Dejan, https://howtomechatronics.com
*/

#include <Arduino.h>
#include "Config.h"
#include <PWMServo.h>

PWMServo ESC;     // create servo object to control the ESC

int potValue = 40;  // value from the analog pin
float time1;
#define DRIVER_MOTOR_PIN  9

void setup() {

    ESC.attach(DRIVER_MOTOR_PIN,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 


for (size_t i = 40; i <= 50; i++)
{
  time1 = (float)millis();
  while ((float)millis() - time1 < 1000)
  {
    time1 = (float)millis();
    ESC.write(i);    // Send the signal to the ESC
    delay(4);
  }
}

/*
  time1 = (float)millis();
  while ((float)millis() - time1 < 10000)
  {
    ESC.write(potValue);    // Send the signal to the ESC
  }
*/
  Serial.begin(115200);
  while (!Serial)
  {
    delay(100);
  }
  
  // Attach the ESC on pin 9

}

void loop() {
    if (potValue > 45)
    {
        potValue = 40;
    }

 potValue += 1;
 time1 = (float)millis();
   while ((float)millis() - time1 < 2000)
  {
    ESC.write(60);    // Send the signal to the ESC
    delay(30);
  }

}


/*

void loop() {
ESC.write(50); 
}
*/