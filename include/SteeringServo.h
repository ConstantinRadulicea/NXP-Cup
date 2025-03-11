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
    int ServoMiddleAngle_calibrated;
    int ServoMaxRightAngle;
    int ServoAngleSpan;

    float SteeringServoAngle;
    float ServoAngleSpan_per_SteeringServoAngle;
    float SteeringServo_MaxLeftAngle;
    float SteeringServo_MaxRightAngle;
    float SteeringAngleOffset;
    float raw_angle_deg;
    /* data */

    void calculateMaxRanges() {

        //this->ServoMaxLeftAngle -= this->SteeringAngleOffset;
        //this->ServoMiddleAngle -= this->SteeringAngleOffset;
        //this->ServoMaxRightAngle -= this->SteeringAngleOffset;

        this->ServoAngleSpan = fabsf(this->ServoMaxRightAngle - this->ServoMaxLeftAngle);
        this->ServoAngleSpan_per_SteeringServoAngle = fabsf((float)this->ServoAngleSpan / (float)fabsf((this->ServoMaxRightAngle - this->ServoMaxLeftAngle)));

        this->SteeringServo_MaxRightAngle = -(float)((float)fabsf(this->ServoMaxRightAngle - this->ServoMiddleAngle_calibrated) / this->ServoAngleSpan_per_SteeringServoAngle);
        this->SteeringServo_MaxLeftAngle = (float)((float)fabsf(this->ServoMaxLeftAngle - this->ServoMiddleAngle_calibrated) / this->ServoAngleSpan_per_SteeringServoAngle);
    }

    float inputAngleToCalibratedRawAngle(float input_angle) {
        int new_servo_angle = this->ServoMiddleAngle_calibrated;
        int cmpResult;

        cmpResult = floatCmp(input_angle, 0.0f);
        if (cmpResult < 0) { // going right
            input_angle = MAX(input_angle, this->SteeringServo_MaxRightAngle);

            input_angle = -input_angle;

            if (this->ServoMaxRightAngle > this->ServoMiddleAngle_calibrated) {
                new_servo_angle = (this->ServoMiddleAngle_calibrated) + (int)(input_angle * this->ServoAngleSpan_per_SteeringServoAngle);
            }
            else {
                new_servo_angle = (this->ServoMiddleAngle_calibrated) - (int)(input_angle * this->ServoAngleSpan_per_SteeringServoAngle);
            }
        }

        else if (cmpResult > 0) {    // going left
            input_angle = MIN(input_angle, this->SteeringServo_MaxLeftAngle);

            if (this->ServoMaxLeftAngle < this->ServoMiddleAngle_calibrated) {
                new_servo_angle = (this->ServoMiddleAngle_calibrated) - (int)((float)(input_angle * this->ServoAngleSpan_per_SteeringServoAngle));
            }
            else {
                new_servo_angle = (this->ServoMiddleAngle_calibrated) + (int)((float)(input_angle * this->ServoAngleSpan_per_SteeringServoAngle));
            }
        }

        else {   // going middle
            new_servo_angle = 0.0f;
        }
        return new_servo_angle;
    }


    float inputAngleToUncalibratedRawAngle(float input_angle) {
        int new_servo_angle = this->ServoMiddleAngle;
        int cmpResult;

        cmpResult = floatCmp(input_angle, 0.0f);
        if (cmpResult < 0) { // going right
            input_angle = MAX(input_angle, this->SteeringServo_MaxRightAngle);

            input_angle = -input_angle;

            if (this->ServoMaxRightAngle > this->ServoMiddleAngle) {
                new_servo_angle = (this->ServoMiddleAngle) + (int)(input_angle * this->ServoAngleSpan_per_SteeringServoAngle);
            }
            else {
                new_servo_angle = (this->ServoMiddleAngle) - (int)(input_angle * this->ServoAngleSpan_per_SteeringServoAngle);
            }
        }

        else if (cmpResult > 0) {    // going left
            input_angle = MIN(input_angle, this->SteeringServo_MaxLeftAngle);

            if (this->ServoMaxLeftAngle < this->ServoMiddleAngle) {
                new_servo_angle = (this->ServoMiddleAngle) - (int)((float)(input_angle * this->ServoAngleSpan_per_SteeringServoAngle));
            }
            else {
                new_servo_angle = (this->ServoMiddleAngle) + (int)((float)(input_angle * this->ServoAngleSpan_per_SteeringServoAngle));
            }
        }

        else {   // going middle
            //new_servo_angle = 0.0f;
        }
        return new_servo_angle;
    }

public:

    SteeringServo()
#ifndef DEBUG_UNITTEST
        : PWMServo()
#endif
    {
        this->SteeringAngleOffset = 0.0f;
        this->SteeringServoAngle = 0;
        this->ServoMaxLeftAngle = (int)0;
        this->ServoMiddleAngle = (int)0;
        this->ServoMiddleAngle_calibrated = (int)0;
        this->ServoMaxRightAngle = (int)0;
        this->ServoAngleSpan = 0.0f;
        this->ServoAngleSpan_per_SteeringServoAngle = 0.0f;

        this->SteeringServo_MaxRightAngle = 0.0f;
        this->SteeringServo_MaxLeftAngle = 0.0f;
        this->raw_angle_deg = 0.0f;
    }

    //                                              130                                    90                                       40
    SteeringServo(unsigned int servo_max_left_angle = 0, unsigned int servo_middle_angle = 90, unsigned int servo_max_right_angle = 180)
#ifndef DEBUG_UNITTEST
        : PWMServo()
#endif
    {
        this->SteeringAngleOffset = 0.0f;
        this->SteeringServoAngle = 0;
        this->ServoMaxLeftAngle = (int)servo_max_left_angle;
        this->ServoMiddleAngle = (int)servo_middle_angle;
        this->ServoMaxRightAngle = (int)servo_max_right_angle;
        this->ServoMiddleAngle_calibrated = this->ServoMiddleAngle;
        this->calculateMaxRanges();
        this->raw_angle_deg = servo_middle_angle;
    }

    ~SteeringServo() {}

    // raw_servo_value = request_angle - offset
    void setMiddleAngleOffset(float offset) {
        this->ServoMiddleAngle_calibrated = this->inputAngleToUncalibratedRawAngle(-offset);
        this->SteeringAngleOffset = offset;
        this->calculateMaxRanges();
    }

    // It interprets the received value as the steering servo angle and converts it to the corresponding angle for the servo motor
    // angle < 0: going right
    // angle > 0: going left
    void setAngleDeg(float steering_angle) {
        int new_servo_angle = this->ServoMiddleAngle_calibrated;
        int cmpResult;


        steering_angle = vaildAngleDeg(steering_angle);
        this->SteeringServoAngle = steering_angle;
        //steering_angle = steering_angle - this->SteeringAngleOffset;
        //steering_angle = vaildAngleDeg(steering_angle);

        new_servo_angle = inputAngleToCalibratedRawAngle(steering_angle);

        this->raw_angle_deg = new_servo_angle;
#ifndef DEBUG_UNITTEST
        this->write(new_servo_angle);
#endif // DEBUG_UNITTEST
    }

    float vaildAngleDeg(float angle) {
        float valid_angle;
        float min_steering_angle;
        float max_steering_angle;
        min_steering_angle = MIN(this->SteeringServo_MaxLeftAngle, this->SteeringServo_MaxRightAngle);
        max_steering_angle = MAX(this->SteeringServo_MaxLeftAngle, this->SteeringServo_MaxRightAngle);

        valid_angle = MAX(MIN(angle, max_steering_angle), min_steering_angle);
        return valid_angle;
    }


    float getAngleDeg() {
        return this->SteeringServoAngle;
    }

    // -1: left, 0: forward, 1: right
    static int AngleToDirectionDeg(float angle) {
        int cmp_result = floatCmp(angle, 0.0);
        if (cmp_result > 0) {
            return -1;
        }
        else if (cmp_result < 0.0) {
            return 1;
        }
        else {
            return 0;
        }
    }


    float getMaxLeftAngleDeg() {
        return SteeringServo_MaxLeftAngle;
    }


    float getMaxRightAngleDeg() {
        return SteeringServo_MaxRightAngle;
    }

    float getRawAngleDeg() {
        return this->raw_angle_deg;
    }
};


#endif