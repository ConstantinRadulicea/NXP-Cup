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