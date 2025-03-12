/*
* Copyright 2024 Constantin Dumitru Petre RĂDULICEA
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

#ifndef __READSERIAL_H__
#define __READSERIAL_H__
#include <Arduino.h>
#include <string>
#include <vector>
#include "Config.h"

static bool readRecordFromSerial(SERIAL_PORT_TYPE &serialPort, String recordTermintor, std::string& record){
  static std::string inputBuffer = std::string();
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
    //record = std::string();
    return false;
    }
    // parse the inputBuffer and load the new global variables
    record = inputBuffer;
    terminatorFound = false;
    inputBuffer.clear();
    return true;
  }
  record = std::string();
  return false;
}


static bool readRecordFromSerial_vector(SERIAL_PORT_TYPE &serialPort, String recordTermintor, std::vector<char>& record){
  static std::vector<char> inputBuffer;
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
    //record = std::string();
    return false;
    }
    // parse the inputBuffer and load the new global variables
    record = inputBuffer;
    terminatorFound = false;
    inputBuffer.clear();
    return true;
  }
  record.clear();
  return false;
}


#endif