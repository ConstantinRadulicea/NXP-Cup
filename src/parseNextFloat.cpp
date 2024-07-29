#include "parseNextFloat.h"

/*
* str:
* C-string beginning with the representation of a floating-point number.
* endptr:
* Reference to an already allocated object of type char*, whose value is set by the function to the next character in str after the numerical value.
* This parameter can also be a null pointer, in which case it is not used.
*/

float parseNextFloat(char* str, size_t strSize, char variableTerminator, char** endptr, int* success) {
	char* nextTerminator;
	char* pEnd;
	float result = 0;

	nextTerminator = (char*)memchr(str, (int)variableTerminator, strSize);
	if (str == nextTerminator && strSize == 1) {
		if (endptr) {
			*endptr = nextTerminator;
		}
		if (success) {
			*success = 0;
		}

		return 0.0f;
	}
	if (nextTerminator) {
		*nextTerminator = '\0';
	}
	else {
		nextTerminator = str + strSize;
	}
	
	result = (float)strtod_(str, &pEnd);  
  

	if (pEnd != nextTerminator) {
		// handle incomplete parse
		pEnd += 1;
		*success = 0;
		if (endptr) {
			*endptr = pEnd;
		}
	}
	else {
		pEnd += 1;
		*success = 1;
		if (endptr) {
			*endptr = pEnd;
		}
	}
	return result;
}