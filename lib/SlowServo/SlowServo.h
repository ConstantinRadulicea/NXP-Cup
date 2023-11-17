#ifndef _SLOWSERVO_H_
#define _SLOWSERVO_H_

#include "Arduino.h"
#include "PWMServo.h"
#include <map>

#define CALLBACK_TIMER 1000*30
#define DEFAULT_ANGLE_INCRESE 4
#define DEFAULT_UPDATE_TIMEOUT 25

class SlowServo : public PWMServo
{
private:
    static unsigned int curID;
    static std::map<unsigned int, SlowServo*> instances;
    static IntervalTimer timer;

    unsigned int InstanceID;
    int finalAngle;
    int tempAngle;
    unsigned int LastUpdateTime_ms;
    unsigned int UpdateTimeout_ms;
    int angleIncrease;


    static void SlowWriteCallback(){
        //Serial.println("Callback: " + String(millis()));
        auto instances_ = SlowServo::GetMapOfActiveObjects();

        for (auto it = instances_.begin(); it != instances_.end(); ++it) {
            it->second->UpdateTempAngle();
            //Serial.println("Callback: local angle=" + String(it->second->tempAngle));
        }
        if (instances_.size() <= 0) {
            SlowServo::GetTimer().end();
        }
    }
    void UpdateTempAngle(){
        if((millis() - LastUpdateTime_ms) >= UpdateTimeout_ms){
            if (finalAngle != tempAngle)
            {
                if (tempAngle > finalAngle)
                {
                    tempAngle = std::max((tempAngle - angleIncrease), finalAngle);
                }
                else{
                    tempAngle = std::min((tempAngle + angleIncrease), finalAngle);
                }
                this->write(tempAngle);
                LastUpdateTime_ms = millis();
            }
            else{
                if (SlowServo::instances.find(InstanceID) != SlowServo::instances.end()) {
                    SlowServo::instances.erase(InstanceID);
                }
            }
        }
    }
public:
    SlowServo() : PWMServo() {
        this->InstanceID = curID++;
        this->finalAngle = 90;
        this->tempAngle = 90;
        this->angleIncrease = DEFAULT_ANGLE_INCRESE;
        this->UpdateTimeout_ms = DEFAULT_UPDATE_TIMEOUT;
        this->LastUpdateTime_ms = 0;
    }

    ~SlowServo(){
        if (SlowServo::instances.find(InstanceID) != SlowServo::instances.end()) {
            SlowServo::instances.erase(InstanceID);
        }
    }

    static std::map<unsigned int, SlowServo*>& GetMapOfActiveObjects() { 
        return instances;
    }

    static IntervalTimer& GetTimer(){
        return timer;
    }

    void SlowWrite(int angleArg){
        this->SlowWrite(angleArg, this->UpdateTimeout_ms, this->angleIncrease);
    }

    void SlowWrite(int angleArg, unsigned int milliseconds, unsigned int angleIncrease){
        if (angleArg == this->tempAngle) {
            this->finalAngle = this->tempAngle;
            return;
        }
        else if(angleIncrease == 0){
            return;
        }
        
        size_t nrInstances;
        this->finalAngle = angleArg;
        this->UpdateTimeout_ms = milliseconds;
        this->angleIncrease = angleIncrease;
        nrInstances = SlowServo::instances.size();
        SlowServo::instances[InstanceID] = this;
        if (nrInstances <= 0) {
            SlowServo::timer.begin(SlowServo::SlowWriteCallback, CALLBACK_TIMER);
        }
    }
    void setUpdateTimeout_ms(unsigned int milliseconds){
        this->UpdateTimeout_ms = milliseconds;
    }

    unsigned int getUpdateTimeout_ms(){
        return this->UpdateTimeout_ms;
    }

    void setAngleIncrease(int angleIncrease){
        this->angleIncrease = angleIncrease;
    }

    int getAngleIncrease(){
        return this->angleIncrease;
    }

    int getTempAngle(){
        return this->tempAngle;
    }
    int getFinalAngle(){
        return this->finalAngle;
    }
};

unsigned int SlowServo::curID = 0;
std::map<unsigned int, SlowServo*> SlowServo::instances;
IntervalTimer SlowServo::timer;

#endif