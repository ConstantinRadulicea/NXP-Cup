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
#include "MedianFilter.h"

#define OBSTACLE_DISTANCE_MEDIANFILTER_SIZE 1

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
    measured_distance = (duration * 0.034321f / 2.0f);

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
  float distance_m = (((float)analogValue * 6.0f) - 300.0f)/1000;
  return distance_m;
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
      estimated_distance_sensor1 = filter_sensor1.next(measured_distance);
      //estimated_distance = measured_distance;
    }
  #endif
  
  #if ENABLE_DISTANCE_SENSOR2 == 1
  if (g_enable_distance_sensor2 != 0)
  {
    analogValue = analogRead(distance_sensor2_analog_pin);
    measured_distance = AnalogToDistance_m(analogValue);
    //estimated_distance_sensor2 = filter_sensor2.next(measured_distance);
    estimated_distance_sensor2 = measured_distance;
  }
  #endif

  #if ENABLE_DISTANCE_SENSOR3 == 1
    if (g_enable_distance_sensor3 != 0)
    {
      analogValue = analogRead(distance_sensor3_analog_pin);
      measured_distance = AnalogToDistance_m(analogValue);
      estimated_distance_sensor3 = filter_sensor3.next(measured_distance);
      //estimated_distance = measured_distance;
    }
  #endif


  estimated_distance = MIN(estimated_distance_sensor1, estimated_distance_sensor2);
  estimated_distance = MIN(estimated_distance, estimated_distance_sensor3);

  return estimated_distance;
}


