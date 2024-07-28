/* FreqMeasureMulti - Example with serial output
 * http://www.pjrc.com/teensy/td_libs_FreqMeasure.html
 *
 * This example code is in the public domain.
 */
#include "Config.h"
#include <FreqMeasureMulti.h>

// Measure 3 frequencies at the same time! :-)
FreqMeasureMulti freq1;

void setup() {
  Serial.begin(230400);
  while (!Serial) ; // wait for Arduino Serial Monitor
  freq1.begin(6);
}

float sum1=0, sum2=0, sum3=0;
int count1=0, count2=0, count3=0;
elapsedMillis timeout;

void loop() {
  if (Serial.available()) {
    uint16_t analog_write_val=0;
    uint16_t val = 0;
    int ch;
    while ((ch = Serial.read()) != -1) {
      if((ch == ' ') || (ch == ',')) {
        analog_write_val= val;
        val = 0;        
      } else if ((ch >= '0') && (ch <= '9')){
        val = val *10 + ch - '0';
      }
    }
  }
  if (freq1.available()) {
    sum1 = sum1 + freq1.read();
    count1 = count1 + 1;
    Serial.println(freq1.countToFrequency(sum1 / count1));
  }

  // print results every half second
  if (timeout > 500) {
    if (count1 > 0) {
      Serial.print(freq1.countToFrequency(sum1 / count1));
    } else {
      Serial.print("(no pulses)");
    }

    Serial.println();
    sum1 = 0;
    sum2 = 0;
    sum3 = 0;
    count1 = 0;
    count2 = 0;
    count3 = 0;
    timeout = 0;
  }
}
