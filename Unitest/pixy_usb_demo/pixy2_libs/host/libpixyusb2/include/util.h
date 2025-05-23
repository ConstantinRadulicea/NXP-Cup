//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#ifndef _UTIL_H
#define _UTIL_H

#define _CRT_SECURE_NO_WARNINGS
#include <stdint.h>

uint32_t millis();
void delayMicroseconds(uint32_t us);

struct Console
{
  void print(const char *msg);
  void println(const char *msg);
};

extern Console Serial;


#endif
