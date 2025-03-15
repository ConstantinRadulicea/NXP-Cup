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

#ifndef __WIFICONNECTION_H__
#define __WIFICONNECTION_H__

#include <HardwareSerial.h>
#include "FloatToString.h"
#include "Config.h"

#define ESP8266_ENABLE_SSDP 1
#define ESP8266_ENABLE_SERVER 1
#define ESP8266_ENABLE_CLIENT 0

#define DEBUG_WIFI_SSID "Off Limits3"
//#define DEBUG_WIFI_SSID "Off Limits2"
#define DEBUG_WIFI_PASSWORD "J7s2tzvzKzva"
//#define DEBUG_WIFI_SSID "Off Limits"
//#define DEBUG_WIFI_PASSWORD "J7s2tzvzKzva"

//#define DEBUG_HOST_IPADDRESS "110.100.0.88"   // Constantin B020
//#define DEBUG_HOST_IPADDRESS "192.168.45.243"   // Constantin phone
//#define DEBUG_HOST_IPADDRESS "192.168.204.243"   // Constantin home
#if CAR_ID == 1
  #define DEBUG_HOST_IPADDRESS "192.168.254.243"   // Daniel phone
#else
  #define DEBUG_HOST_IPADDRESS "192.168.254.243"   // edu
#endif

#define DEBUG_HOST_PORT 6789

#define SSDP_ENABLE_STRING "ENABLE_SSDP"
#define SSDP_DEVICETYPE "urn:schemas-upnp-org:device:NXP-CAR:1"
#define SSDP_NAME "NXP-CUP-CAR"

#if CAR_ID == 1
  #define SSDP_SERIALNUMBER "1"
#elif CAR_ID == 2
    #define SSDP_SERIALNUMBER "2"
#else
    #define SSDP_SERIALNUMBER "0"
#endif

#define SSDP_MODELNAME "Model Name"
#define SSDP_MODELNUMBER "Model Number"
#define SSDP_MODELURL "Model URL"
#define SSDP_MANUFACTURER "Manufacturer"
#define SSDP_MANUFACTURERURL "Manufacturer URL"




#define DEBUG_WIFI_INIT_SEQUENCE "%SERIAL2WIFI\r\n"
#define ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING '%'

#ifndef ESP8266_ENABLE_SERVER
    #define ESP8266_ENABLE_SERVER 0
#endif

#ifndef ESP8266_ENABLE_CLIENT
    #define ESP8266_ENABLE_CLIENT 0
#endif

#if (ESP8266_ENABLE_SERVER == 1 && ESP8266_ENABLE_CLIENT == 1) || (ESP8266_ENABLE_SERVER == 0 && ESP8266_ENABLE_CLIENT == 0)
    #define ESP8266_ENABLE_SERVER 1
#endif

#if ESP8266_ENABLE_SERVER == 1
  #define ESP8266_CONFIGURATION "SERVER"
#endif
  
#if ESP8266_ENABLE_CLIENT == 1
  #define ESP8266_CONFIGURATION "CLIENT"
#endif

#ifndef ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING
    #define ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING ""
#endif

#ifndef ESP8266_ENABLE_SSDP
    #define ESP8266_ENABLE_SSDP 0
#endif


void serial2WifiConnect(SERIAL_PORT_TYPE &serialPort, String initSequence, String wifiSsid, String wifiPassword, String hostname, int port);

/*==============================================================================*/

void printSerial2WifiInfo(SERIAL_PORT_TYPE &serialPort, String initSequence, String wifiSsid, String wifiPassword, String hostname, int port);

void serial2WifiConnectC_str(SERIAL_PORT_TYPE &serialPort, const char* initSequence, const char* wifiSsid, const char* wifiPassword, const char* hostname, int port);

#endif