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

//Methods for setting and getting the steering servo angle

#ifndef _STEERINGSERVO_H_
#define _STEERINGSERVO_H_



#ifndef DEBUG_UNITTEST
#include <PWMServo.h>
#endif // DEBUG_UNITTEST

#include <math.h>
#include "geometry2D.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))



#ifdef DEBUG_UNITTEST
class SteeringServo
#else
class SteeringServo : public PWMServo
#endif // DEBUG_UNITTEST

{
protected:
    int ServoMaxLeftAngle;
    int ServoMiddleAngle;
    int ServoMaxRightAngle;
    int ServoAngleSpan;

    float SteeringServoAngle;
    float ServoAngleSpan_per_SteeringServoAngle;
    float SteeringServo_MaxLeftAngle;
    float SteeringServo_MaxRightAngle;
    float ServoAngle;
    /* data */
public:
    //                                              130                                    90                                       40
    SteeringServo(unsigned int servo_max_left_angle = 0, unsigned int servo_middle_angle = 90, unsigned int servo_max_right_angle = 180)
    #ifndef DEBUG_UNITTEST
        : PWMServo()
    #endif
    {
        this->ServoAngle = servo_middle_angle;
        this->SteeringServoAngle = 0;
        this->ServoMaxLeftAngle = (int) servo_max_left_angle;
        this->ServoMiddleAngle = (int) servo_middle_angle;
        this->ServoMaxRightAngle = (int) servo_max_right_angle;
        this->ServoAngleSpan = fabsf((int)servo_max_right_angle - (int)servo_max_left_angle);
        this->ServoAngleSpan_per_SteeringServoAngle = fabsf((float)this->ServoAngleSpan / (float)fabsf(((int)servo_max_right_angle - (int)servo_max_left_angle)));
        
        this->SteeringServo_MaxRightAngle = -(float)((float)fabsf((int)servo_max_right_angle - (int)servo_middle_angle) / this->ServoAngleSpan_per_SteeringServoAngle);
        this->SteeringServo_MaxLeftAngle = (float)((float)fabsf((int)servo_max_left_angle - (int)servo_middle_angle) / this->ServoAngleSpan_per_SteeringServoAngle);
    }
    ~SteeringServo(){}


    // It interprets the received value as the steering servo angle and converts it to the corresponding angle for the servo motor
    // angle < 0: going right
    // angle > 0: going left
    void setSteeringAngleDeg(float steering_angle){
        int new_servo_angle = this->ServoMiddleAngle;
        int cmpResult;
        steering_angle = vaildSteeringAngleDeg(steering_angle);
        cmpResult = floatCmp(steering_angle, 0.0f);
        if(cmpResult < 0) { // going right
            steering_angle = MAX(steering_angle, this->SteeringServo_MaxRightAngle);
            this->SteeringServoAngle = steering_angle;

            steering_angle = -steering_angle;

            if(this->ServoMaxRightAngle > this->ServoMiddleAngle){
                new_servo_angle = (this->ServoMiddleAngle) + (int)(steering_angle * this->ServoAngleSpan_per_SteeringServoAngle);
            }
            else{
                new_servo_angle = (this->ServoMiddleAngle) - (int)(steering_angle * this->ServoAngleSpan_per_SteeringServoAngle);
            }
        }

        else if(cmpResult > 0){    // going left
            steering_angle = MIN(steering_angle, this->SteeringServo_MaxLeftAngle);
            this->SteeringServoAngle = steering_angle;

            if(this->ServoMaxLeftAngle < this->ServoMiddleAngle){
                new_servo_angle = (this->ServoMiddleAngle) - (int)((float)(steering_angle * this->ServoAngleSpan_per_SteeringServoAngle));
            }
            else{
                new_servo_angle = (this->ServoMiddleAngle) + (int)((float)(steering_angle * this->ServoAngleSpan_per_SteeringServoAngle));
            }
        }

        else{   // going middle
            this->SteeringServoAngle = 0.0;
            new_servo_angle = this->ServoMiddleAngle;
        }
        
        
        #ifndef DEBUG_UNITTEST
            this->write(new_servo_angle);
        #endif // DEBUG_UNITTEST
    }

    float vaildSteeringAngleDeg(float angle){
        float valid_angle;
        float min_steering_angle;
        float max_steering_angle;
        min_steering_angle = MIN(this->SteeringServo_MaxLeftAngle, this->SteeringServo_MaxRightAngle);
        max_steering_angle = MAX(this->SteeringServo_MaxLeftAngle, this->SteeringServo_MaxRightAngle);

        valid_angle = MAX(MIN(angle, max_steering_angle), min_steering_angle);
        return valid_angle;
    }


    float getSteeringAngle(){
        return this->SteeringServoAngle;
    }

    // -1: left, 0: forward, 1: right
    static int AngleToDirectionDeg(float angle){
        int cmp_result = floatCmp(angle, 0.0);
        if (cmp_result > 0) {
            return -1;
        }
        else if(cmp_result < 0.0){
            return 1;
        }
        else {
            return 0;
        }
    }


    float getSteeringServo_MaxLeftAngle(){
        return SteeringServo_MaxLeftAngle;
    }


    float getSteering_MaxRightAngle(){
        return SteeringServo_MaxRightAngle;
    }


    float getServoAngleSpan_per_SteeringServoAngle(){
        return ServoAngleSpan_per_SteeringServoAngle;
    }


    float getServoAngle(){
        if(ServoMaxRightAngle > ServoMiddleAngle){  // going right
            return this->ServoMiddleAngle - this->ServoAngle;
            //return ServoMiddleAngle - this->read();
        }
        else{                                       // going left
            return this->ServoAngle - this->ServoMiddleAngle;
            //return this->read() - ServoMiddleAngle;
        }
        
    }
};


#endif