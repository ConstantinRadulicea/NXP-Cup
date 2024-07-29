#ifndef __PARSENEXTFLOAT_H__
#define __PARSENEXTFLOAT_H__
#include "strtod_.h"
#include <memory.h>

/*
* str:
* C-string beginning with the representation of a floating-point number.
* endptr:
* Reference to an already allocated object of type char*, whose value is set by the function to the next character in str after the numerical value.
* This parameter can also be a null pointer, in which case it is not used.
*/

float parseNextFloat(char* str, size_t strSize, char variableTerminator, char** endptr, int* success) ;

#endif