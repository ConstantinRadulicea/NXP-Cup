// used for testing in matlab

#ifndef _PARSENUM_H_
#define _PARSENUM_H_
#include <Arduino.h>

String input_buffer;

// 0: message not complete
// 1: message complete
int ReadSerial() //reading form serial port into input_buffer
{
  int i = 0;
  int input_char;
    while (Serial.available() > 0) {//while there is data to read
      i++;
        input_char = Serial.read();
        if (input_char == '@') {// a convention to reset the buffer to distinguish between data
          input_buffer = "";
        }
        else if(input_char == '\n') {// returns the number of characters in the message
          Serial.println("% SerialRead chars: " + String(i));
          return 1;
        }
        else{
          input_buffer += (char)input_char;// construct the message
        }
    }
    Serial.println("% SerialRead chars: " + String(i));
    return 0;
}

// returns the number of digits and the position
int ReadNextInt(const char* _str, int* integer){
  int sign = 1;
  int i = 0;
  int num = 0;
  char character = '\0';

  if (_str[i] == '-'){
    sign = -1;
    i++;
  }
  else if (_str[i] == '+') {
    i++;
  }
  
  while (_str[i])
  {
    character = _str[i];
    if(! isDigit(character)){
      *integer = num * sign;
      return i;
    }
    num *= 10;
    num += (character - '0');
    i++;
  }
  *integer = num * sign;
  return i;
}

// returns the number of chars read and the position
int ReadNextFloat(const char* _str, float* integer){
  int sign = 1;
  int sub_unitar_digits = 0;
  int sub_unitar_num = 0;
  int unitar_num = 0;
  int i = 0;
  float num = 0;
  char character = '\0';

  if (_str[i] == '-'){
    sign = -1;
    i++;
  }
  else if (_str[i] == '+') {
    i++;
  }
  while (_str[i])
  {
    character = _str[i];
    if(!isDigit(character)){
      if (character == '.' && (_str[i+1] != '-' && _str[i+1] != '+'))
      {
        sub_unitar_digits = ReadNextInt(&(_str[i+1]), &sub_unitar_num);
        i += sub_unitar_digits + 1;
        num = unitar_num;
        num += (sub_unitar_num / pow(10, sub_unitar_digits));
        *integer = num * sign;
        return i;
      }
      else{
        *integer = unitar_num * sign;
        return i;
      }
    }
    unitar_num *= 10;
    unitar_num += (character - '0');
    i++;
  }
  *integer = unitar_num * sign;
  return i;
}



#endif