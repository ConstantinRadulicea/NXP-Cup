#include "WifiConnection.h"




void serial2WifiConnect(SERIAL_PORT_TYPE &serialPort, String initSequence, String wifiSsid, String wifiPassword, String hostname, int port){
    String commentChar = String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING);
    serialPort.print(initSequence);
    serialPort.println(commentChar + String(ESP8266_CONFIGURATION));
    serialPort.println(commentChar + wifiSsid);
    serialPort.println(commentChar + wifiPassword);
    #if ESP8266_ENABLE_CLIENT != 0
      Serial.println(commentChar + String(hostname));
    #endif
    serialPort.println(commentChar + FloatToString((float)port, 0));
  
      #if ESP8266_ENABLE_SSDP != 0
        serialPort.println(commentChar + String(SSDP_ENABLE_STRING));
        serialPort.println(commentChar + String(SSDP_DEVICETYPE));
        serialPort.println(commentChar + String(SSDP_NAME));
        serialPort.println(commentChar + String(SSDP_SERIALNUMBER));
        serialPort.println(commentChar + String(SSDP_MODELNAME));
        serialPort.println(commentChar + String(SSDP_MODELNUMBER));
        serialPort.println(commentChar + String(SSDP_MODELURL));
        serialPort.println(commentChar + String(SSDP_MANUFACTURER));
        serialPort.println(commentChar + String(SSDP_MANUFACTURERURL));
      #else
        serialPort.println(commentChar + String("DISABLE_SSDP"));
      #endif
  }

  void serial2WifiConnectC_str(SERIAL_PORT_TYPE &serialPort, const char* initSequence, const char* wifiSsid, const char* wifiPassword, const char* hostname, int port){
    char commentChar = ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING;
    serialPort.print(initSequence);
    serialPort.print(commentChar);
    serialPort.println(ESP8266_CONFIGURATION);
    //serialPort.println(commentChar + String("CLIENT"));
    serialPort.print(commentChar);
    serialPort.println(wifiSsid);

    serialPort.print(commentChar);
    serialPort.println(wifiPassword);

    #if ESP8266_ENABLE_CLIENT != 0
        serialPort.print(commentChar);
        serialPort.println(hostname);
    #endif
        serialPort.print(commentChar);
        serialPort.println(FloatToC_str((float)port, 0));
    
  
      #if ESP8266_ENABLE_SSDP != 0
        serialPort.print(commentChar);
        serialPort.println(SSDP_ENABLE_STRING);

        serialPort.print(commentChar);
        serialPort.println(SSDP_DEVICETYPE);

        serialPort.print(commentChar);
        serialPort.println(SSDP_NAME);

        serialPort.print(commentChar);
        serialPort.println(SSDP_SERIALNUMBER);

        serialPort.print(commentChar);
        serialPort.println(SSDP_MODELNAME);

        serialPort.print(commentChar);
        serialPort.println(SSDP_MODELNUMBER);

        serialPort.print(commentChar);
        serialPort.println(SSDP_MODELURL);

        serialPort.print(commentChar);
        serialPort.println(SSDP_MANUFACTURER);

        serialPort.print(commentChar);
        serialPort.println(SSDP_MANUFACTURERURL);
      #else
        serialPort.print(commentChar);
        serialPort.println("DISABLE_SSDP");
      #endif
  }
  
  /*==============================================================================*/
  
 void printSerial2WifiInfo(SERIAL_PORT_TYPE &serialPort, String initSequence, String wifiSsid, String wifiPassword, String hostname, int port){
    
    String commentChar = String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING);
    serialPort.print(commentChar + String("WIFI INIT SEQUENCE: ") + initSequence);
    serialPort.println(commentChar + String("SSID: ") + wifiSsid);
    serialPort.println(commentChar + String("PASSWORD: ") + wifiPassword);
    serialPort.println(commentChar + String("HOSTNAME: ") + hostname);
    serialPort.println(commentChar + String("PORT: ") + FloatToString((float)port, 0));
  
  }
  