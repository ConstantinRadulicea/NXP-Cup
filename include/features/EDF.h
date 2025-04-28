#ifndef __EDF_H__
#define __EDF_H__

#include <PWMServo.h>

extern volatile PWMServo EDF_motor;

void EDF_setup(int edf_pin);

void EDF_activation_loop(float steering_angle_rad);



#endif