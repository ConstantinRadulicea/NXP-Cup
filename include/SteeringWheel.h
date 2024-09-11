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

//Methods for setting and getting the steering wheel angle

#ifndef _STEERINGWHEEL_H_
#define _STEERINGWHEEL_H_

#include <PWMServo.h>
#include <math.h>
#include <geometry2D.h>

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

class SteeringWheel : public PWMServo
{
private:
    int ServoMaxLeftAngle;
    int ServoMiddleAngle;
    int ServoMaxRightAngle;
    int ServoAngleSpan;

    float SteeringWheelAngle;
    float ServoAngleSpan_per_SteeringWheelAngle;
    float SteeringWheel_MaxLeftAngle;
    float SteeringWheel_MaxRightAngle;
    /* data */
public:
    //                                              130                                    90                                       40
    SteeringWheel(unsigned int servo_max_left_angle = 0, unsigned int servo_middle_angle = 90, unsigned int servo_max_right_angle = 180, unsigned int milliseconds = 0) : PWMServo(){
        this->SteeringWheelAngle = 0;
        this->ServoMaxLeftAngle = (int) servo_max_left_angle;
        this->ServoMiddleAngle = (int) servo_middle_angle;
        this->ServoMaxRightAngle = (int) servo_max_right_angle;
        this->ServoAngleSpan = abs((int)servo_max_right_angle - (int)servo_max_left_angle);
        this->ServoAngleSpan_per_SteeringWheelAngle = abs((float)this->ServoAngleSpan / (float)abs(((int)servo_max_right_angle - (int)servo_max_left_angle)));
        
        this->SteeringWheel_MaxRightAngle = -(float)((float)abs((int)servo_max_right_angle - (int)servo_middle_angle) / this->ServoAngleSpan_per_SteeringWheelAngle);
        this->SteeringWheel_MaxLeftAngle = (float)((float)abs((int)servo_max_left_angle - (int)servo_middle_angle) / this->ServoAngleSpan_per_SteeringWheelAngle);
    }
    ~SteeringWheel(){}


    // It interprets the received value as the steering wheel angle and converts it to the corresponding angle for the servo motor
    // angle < 0: going right
    // angle > 0: going left
    void setSteeringAngleDeg(float steering_angle){
        int new_servo_angle = this->ServoMiddleAngle;

        if(steering_angle < 0) { // going right
            steering_angle = MAX(steering_angle, this->SteeringWheel_MaxRightAngle);
            this->SteeringWheelAngle = steering_angle;

            steering_angle = -steering_angle;

            if(this->ServoMaxRightAngle > this->ServoMiddleAngle){
                new_servo_angle = (this->ServoMiddleAngle) + (int)(steering_angle * this->ServoAngleSpan_per_SteeringWheelAngle);
            }
            else{
                new_servo_angle = (this->ServoMiddleAngle) - (int)(steering_angle * this->ServoAngleSpan_per_SteeringWheelAngle);
            }
        }

        else if(steering_angle > 0){    // going left
            steering_angle = MIN(steering_angle, this->SteeringWheel_MaxLeftAngle);
            this->SteeringWheelAngle = steering_angle;

            if(this->ServoMaxLeftAngle < this->ServoMiddleAngle){
                new_servo_angle = (this->ServoMiddleAngle) - (int)((float)(steering_angle * this->ServoAngleSpan_per_SteeringWheelAngle));
            }
            else{
                new_servo_angle = (this->ServoMiddleAngle) + (int)((float)(steering_angle * this->ServoAngleSpan_per_SteeringWheelAngle));
            }
        }

        else{   // going middle
            this->SteeringWheelAngle = 0;
            new_servo_angle = this->ServoMiddleAngle;
        }
        this->write(new_servo_angle);
    }

    float vaildSteeringAngleDeg(float angle){
        float valid_angle;
        float min_steering_angle;
        float max_steering_angle;
        min_steering_angle = MIN(this->SteeringWheel_MaxLeftAngle, this->SteeringWheel_MaxRightAngle);
        max_steering_angle = MAX(this->SteeringWheel_MaxLeftAngle, this->SteeringWheel_MaxRightAngle);

        valid_angle = MAX(MIN(angle, max_steering_angle), min_steering_angle);
        return valid_angle;
    }


    float getSteeringAngle(){
        return this->SteeringWheelAngle;
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


    float getSteeringWheel_MaxLeftAngle(){
        return SteeringWheel_MaxLeftAngle;
    }


    float getSteering_MaxRightAngle(){
        return SteeringWheel_MaxRightAngle;
    }


    float getServoAngleSpan_per_SteeringWheelAngle(){
        return ServoAngleSpan_per_SteeringWheelAngle;
    }


    float getWheelAngle(){
        if(ServoMaxRightAngle > ServoMiddleAngle){  // going right
            return ServoMiddleAngle - this->read();
        }
        else{                                       // going left
            return this->read() - ServoMiddleAngle;
        }
        
    }
};


#endif