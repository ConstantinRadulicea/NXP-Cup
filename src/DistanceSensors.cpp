/*
* Copyright 2023 Constantin Dumitru Petre RĂDULICEA
* 
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*   http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include "DistanceSensors.h"

IntervalTimer myTimer_2;

#define OBSTACLE_DISTANCE_MEDIANFILTER_SIZE 3


int distance_sensor1_trig_pin, distance_sensor1_echo_pin;
int distance_sensor2_trig_pin, distance_sensor2_echo_pin;
int distance_sensor3_trig_pin, distance_sensor3_echo_pin;

int distance_sensor1_analog_pin;
int distance_sensor2_analog_pin;
int distance_sensor3_analog_pin;

void DistanceSensorsSetupHCSR04(
int _distance_sensor1_trig_pin, int _distance_sensor1_echo_pin,
int _distance_sensor2_trig_pin, int _distance_sensor2_echo_pin,
int _distance_sensor3_trig_pin, int _distance_sensor3_echo_pin)
{
distance_sensor1_trig_pin = _distance_sensor1_trig_pin;
distance_sensor2_trig_pin = _distance_sensor2_trig_pin;
distance_sensor3_trig_pin = _distance_sensor3_trig_pin;

distance_sensor1_echo_pin = _distance_sensor1_echo_pin;
distance_sensor2_echo_pin = _distance_sensor2_echo_pin;
distance_sensor3_echo_pin = _distance_sensor3_echo_pin;

  #if ENABLE_DISTANCE_SENSOR1 == 1
    pinMode(distance_sensor1_trig_pin, OUTPUT); 
    pinMode(distance_sensor1_echo_pin, INPUT); 
  #endif

  #if ENABLE_DISTANCE_SENSOR2 == 1
    pinMode(distance_sensor2_trig_pin, OUTPUT); 
    pinMode(distance_sensor2_echo_pin, INPUT); 
  #endif

  #if ENABLE_DISTANCE_SENSOR3 == 1
    pinMode(distance_sensor3_trig_pin, OUTPUT); 
    pinMode(distance_sensor3_echo_pin, INPUT); 
  #endif

}



/*
https://maxbotix.com/blogs/blog/using-multiple-ultrasonic-sensors?srsltid=AfmBOop7wNWDCYEO1pBa_BS5Sa0NgRTmjuTkSKhbaqd6VbN60W_Q2xZz
To enable the AN Output Constantly Looping:
connect (Pin 2- Pulse Width Output) to (Pin 4- Ranging Start/Stop)

*/


void continous_loop_signal_generation(int signal_pin, int8_t _count) {
    if (_count == (int8_t)1) {
        digitalWrite(signal_pin, HIGH);
    }
    else {
        digitalWrite(signal_pin, LOW);
    }
}

void continous_loop_signal_generation_routine(){
  static int8_t _count = 0;
  //continous_loop_signal_generation(EDF_MOTOR_PIN, _count);
  continous_loop_signal_generation(EDF_MOTOR_PIN, _count);
  //continous_loop_signal_generation(EDF_MOTOR_PIN, _count);
  _count++;
  _count = _count % (int8_t)3;
}


void DistanceSensorsSetupAnalog(
int _distance_sensor1_analog_pin,
int _distance_sensor2_analog_pin,
int _distance_sensor3_analog_pin){

distance_sensor1_analog_pin = _distance_sensor1_analog_pin;
distance_sensor2_analog_pin = _distance_sensor2_analog_pin;
distance_sensor3_analog_pin = _distance_sensor3_analog_pin;

  #if ENABLE_DISTANCE_SENSOR1 == 1
    pinMode(distance_sensor1_analog_pin, INPUT);
  #endif

  #if ENABLE_DISTANCE_SENSOR2 == 1
    pinMode(distance_sensor2_analog_pin, INPUT); 
  #endif

  #if ENABLE_DISTANCE_SENSOR3 == 1
    pinMode(distance_sensor3_analog_pin, INPUT); 
  #endif

  pinMode(EDF_MOTOR_PIN, OUTPUT);
  digitalWrite(EDF_MOTOR_PIN, LOW);

  myTimer_2.begin(continous_loop_signal_generation_routine, SecToMicros(HzToSec(30.0)));

}

float getFrontObstacleDistanceHCSR04_m(){
  //static SimpleKalmanFilter simpleKalmanFilter(0.1f, 0.1f, 0.001f);

  #if ENABLE_DISTANCE_SENSOR1 == 1
    static MedianFilter filter_sensor1(OBSTACLE_DISTANCE_MEDIANFILTER_SIZE);
  #endif
  #if ENABLE_DISTANCE_SENSOR2 == 1
    static MedianFilter filter_sensor2(OBSTACLE_DISTANCE_MEDIANFILTER_SIZE);
  #endif

  #if ENABLE_DISTANCE_SENSOR3 == 1
    static MedianFilter filter_sensor3(OBSTACLE_DISTANCE_MEDIANFILTER_SIZE);
  #endif
  
  // calculations were made in centimeters
  static uint32_t pulseInTimeout_us = (uint32_t)((200.0f / 34300.0f) * 1000000.0f);

  float duration;
  float measured_distance = 0.0f;
  float estimated_distance = 0.0f;
  float estimated_distance_sensor1 = 400.0f, estimated_distance_sensor2 = 400.0f, estimated_distance_sensor3 = 400.0f;

  #if ENABLE_DISTANCE_SENSOR1 == 1
    if (g_enable_distance_sensor1 != 0)
    {
      digitalWrite(distance_sensor1_trig_pin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(distance_sensor1_trig_pin, HIGH);
      delayMicroseconds(10); //This pin should be set to HIGH for 10 μs, at which point the HC­SR04 will send out an eight cycle sonic burst at 40 kHZ
      digitalWrite(distance_sensor1_trig_pin, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = (float)(pulseIn(distance_sensor1_echo_pin, HIGH, pulseInTimeout_us));
      // Calculating the distance
      measured_distance = duration * 0.034321f / 2.0f;

      if (measured_distance <= 0.0f) {
        measured_distance = 400.0f;
      }

      measured_distance = MIN(measured_distance, 400.0f);

      //estimated_distance = simpleKalmanFilter.updateEstimate(measured_distance);
      estimated_distance_sensor1 = filter_sensor1.next(measured_distance);
      //estimated_distance = measured_distance;
    }
  #endif

  #if ENABLE_DISTANCE_SENSOR1 == 1 && ENABLE_DISTANCE_SENSOR2 == 1
  if ((g_enable_distance_sensor1 != 0) && (g_enable_distance_sensor2 != 0)) {
    delay(1);
  }
  #endif
  
  #if ENABLE_DISTANCE_SENSOR2 == 1
  if (g_enable_distance_sensor2 != 0)
  {
    
    digitalWrite(distance_sensor2_trig_pin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(distance_sensor2_trig_pin, HIGH);
    delayMicroseconds(10); //This pin should be set to HIGH for 10 μs, at which point the HC­SR04 will send out an eight cycle sonic burst at 40 kHZ
    digitalWrite(distance_sensor2_trig_pin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = (float)(pulseIn(distance_sensor2_echo_pin, HIGH, pulseInTimeout_us));
    // Calculating the distance
    measured_distance = ((duration * 0.034321f) / 2.0f);

    if (measured_distance <= 0.0f) {
      measured_distance = 400.0f;
    }

    measured_distance = MIN(measured_distance, 400.0f);

    //estimated_distance = simpleKalmanFilter.updateEstimate(measured_distance);
    estimated_distance_sensor2 = filter_sensor2.next(measured_distance);
    //estimated_distance = measured_distance;
  }
  #endif


  #if ENABLE_DISTANCE_SENSOR3 == 1 && (ENABLE_DISTANCE_SENSOR2 == 1 || ENABLE_DISTANCE_SENSOR1 == 1)
  if ((g_enable_distance_sensor3 != 0) && ((g_enable_distance_sensor1 != 0) || (g_enable_distance_sensor2 != 0))) {
    delay(1);
  }
  #endif

  #if ENABLE_DISTANCE_SENSOR3 == 1
    if (g_enable_distance_sensor3 != 0)
    {
      digitalWrite(distance_sensor3_trig_pin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(distance_sensor3_trig_pin, HIGH);
      delayMicroseconds(10); //This pin should be set to HIGH for 10 μs, at which point the HC­SR04 will send out an eight cycle sonic burst at 40 kHZ
      digitalWrite(distance_sensor3_trig_pin, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = (float)(pulseIn(distance_sensor3_echo_pin, HIGH, pulseInTimeout_us));
      // Calculating the distance
      measured_distance = duration * 0.034321f / 2.0f;

      if (measured_distance <= 0.0f) {
        measured_distance = 400.0f;
      }

      measured_distance = MIN(measured_distance, 400.0f);

      //estimated_distance = simpleKalmanFilter.updateEstimate(measured_distance);
      estimated_distance_sensor3 = filter_sensor3.next(measured_distance);
      //estimated_distance = measured_distance;
    }
  #endif


  estimated_distance = MIN(estimated_distance_sensor1, estimated_distance_sensor2);
  estimated_distance = MIN(estimated_distance, estimated_distance_sensor3);

  return estimated_distance / 100.0f;
}


float AnalogToDistance_m(int analogValue){
  // distance_mm = ([V_observed / (Vcc/ 1024)] * 6) - 300
  float distance_m = (((float)analogValue * 6.0f) - 300.0f)/1000.0;
  return distance_m;
}

// calibrated_data = sensor_data - offset		
// offset =	sensor_data * x + c
float CalibrateDistance_linear(float sensor_data, float x, float c){
  float calibrated_data, offset;
  offset = (sensor_data * x) + c;
  calibrated_data = sensor_data - offset;
  return calibrated_data;
}

float getFrontObstacleDistanceAnalog_m(){
  //static SimpleKalmanFilter simpleKalmanFilter(0.1f, 0.1f, 0.001f);

  #if ENABLE_DISTANCE_SENSOR1 == 1
    static MedianFilter filter_sensor1(OBSTACLE_DISTANCE_MEDIANFILTER_SIZE);
  #endif
  #if ENABLE_DISTANCE_SENSOR2 == 1
    static MedianFilter filter_sensor2(OBSTACLE_DISTANCE_MEDIANFILTER_SIZE);
  #endif

  #if ENABLE_DISTANCE_SENSOR3 == 1
    static MedianFilter filter_sensor3(OBSTACLE_DISTANCE_MEDIANFILTER_SIZE);
  #endif

  int analogValue;
  float measured_distance = 0.0f;
  float estimated_distance = 0.0f;
  float estimated_distance_sensor1 = 5.0f, estimated_distance_sensor2 = 5.0f, estimated_distance_sensor3 = 5.0f;

  #if ENABLE_DISTANCE_SENSOR1 == 1
    if (g_enable_distance_sensor1 != 0)
    {
      analogValue = analogRead(distance_sensor1_analog_pin);
      measured_distance = AnalogToDistance_m(analogValue);
      measured_distance = CalibrateDistance_linear(measured_distance, DISTANCE_SENSOR1_CALIBRATION_FORMULA_X, DISTANCE_SENSOR1_CALIBRATION_FORMULA_C);
      estimated_distance_sensor1 = filter_sensor1.next(measured_distance);
      estimated_distance_sensor1 = estimated_distance_sensor1 - DISTANCE_SENSOR1_OFFSET_M;
    }
  #endif
  
  #if ENABLE_DISTANCE_SENSOR2 == 1
  if (g_enable_distance_sensor2 != 0)
  {
    analogValue = analogRead(distance_sensor2_analog_pin);
    measured_distance = AnalogToDistance_m(analogValue);
    measured_distance = CalibrateDistance_linear(measured_distance, DISTANCE_SENSOR2_CALIBRATION_FORMULA_X, DISTANCE_SENSOR2_CALIBRATION_FORMULA_C);
    estimated_distance_sensor2 = filter_sensor2.next(measured_distance);
    estimated_distance_sensor2 = estimated_distance_sensor2 - DISTANCE_SENSOR2_OFFSET_M;
  }
  #endif

  #if ENABLE_DISTANCE_SENSOR3 == 1
    if (g_enable_distance_sensor3 != 0)
    {
      analogValue = analogRead(distance_sensor3_analog_pin);
      measured_distance = AnalogToDistance_m(analogValue);
      measured_distance = CalibrateDistance_linear(measured_distance, DISTANCE_SENSOR3_CALIBRATION_FORMULA_X, DISTANCE_SENSOR3_CALIBRATION_FORMULA_C);
      estimated_distance_sensor3 = filter_sensor3.next(measured_distance);
      estimated_distance_sensor3 = estimated_distance_sensor3 - DISTANCE_SENSOR3_OFFSET_M;
    }
  #endif


  estimated_distance = MIN(estimated_distance_sensor1, estimated_distance_sensor2);
  estimated_distance = MIN(estimated_distance, estimated_distance_sensor3);

  return estimated_distance;
}


