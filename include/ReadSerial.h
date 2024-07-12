#ifndef __READSERIAL_H__
#define __READSERIAL_H__
#include <Arduino.h>
#include <vector>

template<typename T> bool readRecordFromSerial(T& serialPort, String recordTermintor, std::vector<char> &record){
  static std::vector<char> inputBuffer = std::vector<char>();
  static bool terminatorFound = false;

  char tempChar, lastTerminatorCharacter;

  if (serialPort.available() <= 0) {
    //record = std::vector<char>();
    return false;
  }

  if (terminatorFound == true && inputBuffer.size() > 0) {
    terminatorFound = false;
    inputBuffer.clear();
  }
  

  lastTerminatorCharacter = recordTermintor.charAt(recordTermintor.length()-1);

  while (serialPort.available() > 0){
    tempChar = (char)serialPort.read();
    inputBuffer.push_back(tempChar);
    if (tempChar == lastTerminatorCharacter && inputBuffer.size() >= (size_t)recordTermintor.length()) {
      if (memcmp((const void*)recordTermintor.c_str(), (const void*)(inputBuffer.data() + (size_t)((int)inputBuffer.size() - (int)recordTermintor.length())), (size_t)recordTermintor.length()) == 0) {
        terminatorFound = true;
        break;
      }
    }
  }

  if (terminatorFound) {

    if (inputBuffer.size() >= (size_t)recordTermintor.length()) {
      inputBuffer.erase(inputBuffer.begin() + (inputBuffer.size() - (size_t)recordTermintor.length()), inputBuffer.end());
    }
    else {
      // If vector has less than n number of elements,
      // then delete all elements
      inputBuffer.clear();
    //record = std::vector<char>();
    return false;
    }
    // parse the inputBuffer and load the new global variables
    record = inputBuffer;
    record.push_back('\0');
    terminatorFound = false;
    inputBuffer.clear();
    return true;
  }
  record = std::vector<char>();
  return false;
}


#endif