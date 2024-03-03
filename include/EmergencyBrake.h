#ifndef __EMERGENCYBRAKE_H__
#define __EMERGENCYBRAKE_H__

#include "MovingAverage.h"
#include <Arduino.h>

#include <PWMServo.h>


class ObstacleDistance
{
private:
    int TRIG_PIN, ECHO_PIN;
    MovingAverage movingAverage;

    float getFrontObstacleDistance_cm(){
        float duration;
        float measured_distance;
        float estimated_distance;

        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10); //This pin should be set to HIGH for 10 μs, at which point the HC­SR04 will send out an eight cycle sonic burst at 40 kHZ
        digitalWrite(TRIG_PIN, LOW);
        // Reads the echoPin, returns the sound wave travel time in microseconds
        duration = (float)(pulseIn(ECHO_PIN, HIGH));
        // Calculating the distance
        measured_distance = duration * 0.034321f / 2.0f;

        if (measured_distance <= 0.0f) {
            measured_distance = 400.0f;
        }
        
        measured_distance = MIN(measured_distance, 400.0f);

        //estimated_distance = simpleKalmanFilter.updateEstimate(measured_distance);
        estimated_distance = movingAverage.next(measured_distance);
        //estimated_distance = measured_distance;

        return estimated_distance;
    }

public:
    ObstacleDistance(int TRIG_PIN, int ECHO_PIN) : movingAverage(4) {
        this->TRIG_PIN = TRIG_PIN;
        this->ECHO_PIN = ECHO_PIN;
    }

    float getDistance(){
        return this->getFrontObstacleDistance_cm();
    }
    
    void emergencyBrake(PWMServo &driverMotor, float startEmergencyBrakeDistance_cm, float distanceToStopFromObstacle_cm, float standstillSpeed, float maxForwardSpeed, float maxBacwardSpeed){
        float currentSpeed, currentObstacleDistance, prevObjectDistance, currentSpeed_m_s, prevSpeed_m_s, deltaDistance_m;
        float timeStart_us, timeEnd_us, deltaTime_s;

        currentObstacleDistance = this->getDistance();
        timeStart_us = (float)micros();

        currentSpeed = driverMotor.read();
        prevObjectDistance = currentObstacleDistance;

        delay(2);
        currentObstacleDistance = this->getDistance();
        timeEnd_us = (float)micros();

        deltaTime_s = (timeEnd_us - timeStart_us) / 1000000.0f;
        deltaDistance_m = (prevObjectDistance - currentObstacleDistance) / 100.0f;
        currentSpeed_m_s = deltaDistance_m / deltaTime_s;
        prevSpeed_m_s = currentSpeed_m_s;

        // start emergency braking
        while (currentObstacleDistance <= startEmergencyBrakeDistance_cm)
        {
            timeStart_us = (float)micros();
            prevObjectDistance = currentObstacleDistance;
            currentObstacleDistance = this->getDistance();

            currentSpeed = driverMotor.read();

            deltaTime_s = (timeEnd_us - timeStart_us) / 1000000.0f;
            deltaDistance_m = (prevObjectDistance - currentObstacleDistance) / 100.0f;
            currentSpeed_m_s = deltaDistance_m / deltaTime_s;
            prevSpeed_m_s = currentSpeed_m_s;



            if (currentObstacleDistance <= distanceToStopFromObstacle_cm) {
                driverMotor.write((int)standstillSpeed);
            }
            timeEnd_us = (float)micros();
            deltaTime_s = (timeEnd_us - timeStart_us) / 1000000.0f;
        }
        
    }
    ~ObstacleDistance();
};

#endif