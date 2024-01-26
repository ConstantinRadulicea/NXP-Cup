//Methods for setting and getting the steering wheel angle

#ifndef _STEERINGWHEEL_H_
#define _STEERINGWHEEL_H_

#include "SlowServo.h" // smoother control over the servo

class SteeringWheel : public SlowServo
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
    SteeringWheel(unsigned int servo_max_left_angle = 0, unsigned int servo_middle_angle = 90, unsigned int servo_max_right_angle = 180, unsigned int milliseconds = 0) : SlowServo(){
        this->setUpdateTimeout_ms(milliseconds);
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


    void setSteeringAngleDeg(float steering_angle){// It interprets the received value as the steering wheel angle and converts it to the corresponding angle for the servo motor
        int new_servo_angle = 90;

        if(steering_angle < 0){ // going right
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
        this->SlowWrite(new_servo_angle);
    }


    float getSteeringAngle(){
        return this->SteeringWheelAngle;
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
            return ServoMiddleAngle - this->getTempAngle();
        }
        else{                                       // going left
            return this->getTempAngle() - ServoMiddleAngle;
        }
        
    }
};


#endif