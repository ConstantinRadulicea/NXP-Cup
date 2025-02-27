#include "setup.h"



#if (ENABLE_SERIAL_PRINT == 1 || ENABLE_WIRELESS_DEBUG == 1) && SERIAL_PORT_TYPE_CONFIGURATION == 1 && defined(TEENSY40)
  #define ENABLE_SERIAL_BUFFER
  #define RX_BUFFER_SIZE 4092
  #define TX_BUFFER_SIZE 16384
  static char RX_BUFFER[RX_BUFFER_SIZE];
  static char TX_BUFFER[TX_BUFFER_SIZE];
#endif



void FailureModeMessage(Pixy2 *pixy, float time_passed, String errorText){
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("seconds [") + String(time_passed) + String("] ERROR: " + errorText));
  #endif
  if (time_passed >= CAMERA_ERROR_TIMEOUT_S){
    //g_car_speed_mps = (float)STANDSTILL_SPEED;
    while (pixy->init() < ((int8_t)0))
    {
      #if ENABLE_DRIVERMOTOR == 1
        #if ENABLE_SINGLE_AXE_STEERING_NO_RPM != 0
          g_onemotorpowertrain.SetSpeedRequest_slow(STANDSTILL_SPEED);
        #else
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            g_powertrain.SetSpeedRequest_slow(STANDSTILL_SPEED, 0.0, 0, g_max_acceleration, g_max_deceleration);
          }
        #endif
      #endif
      g_steering_angle_rad = 0.0f;
      #if ENABLE_STEERING_SERVO == 1
        g_steering_wheel.setSteeringWheelAngleDeg(0.0f);
      #endif
      delay(10);
    }    
  }
}


int isValidFloatNumber(float *num, int line){
    if (!isfinite(*num)) {
      #if ENABLE_SERIAL_PRINT == 1
          SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("ERROR: isfinite at line ") + String(line));
      #endif
      return 0;
    }
    return 1;
}



void setup() {
  int8_t pixyResult;
  //Serial.begin(SERIAL_PORT_BAUD_RATE);
  // Initialization and attachment of the servo and motor
  g_steering_angle_rad = 0.0f;
  #if ENABLE_STEERING_SERVO == 1
    pinMode(STEERING_SERVO_PIN, OUTPUT);
    g_steering_wheel.steering_servo.attach(STEERING_SERVO_PIN, 500, 2500);
    g_steering_wheel.SetRawAngleOffset(g_steering_wheel_angle_offset_deg);
    g_steering_wheel.setSteeringWheelAngleDeg(0.0f);
  #endif

  #if ENABLE_DRIVERMOTOR == 1
    #if ENABLE_SINGLE_AXE_STEERING_NO_RPM != 0
      OneMotorPowerTrainSetup(WHEEL_DIAMETER_M, LEFT_WHEEL_MOTOR_PIN);
    #else
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        PowerTrainSetup(WHEEL_DIAMETER_M, TRACK_WIDTH_M, POWERTRAIN_PID_FREQUENCY_HZ, LEFT_WHEEL_MOTOR_PIN, RIGHT_WHEEL_MOTOR_PIN, RPM_SENSOR_LEFT_WHEEL_PIN, RPM_SENSOR_RIGHT_WHEEL_PIN);
        g_powertrain.SetLeftWheelPID(g_powertrain_left_wheel_kp, g_powertrain_left_wheel_ki, g_powertrain_left_wheel_kd, g_powertrain_left_wheel_ki_max_sum);
        g_powertrain.SetRightWheelPID(g_powertrain_right_wheel_kp, g_powertrain_right_wheel_ki, g_powertrain_right_wheel_kd, g_powertrain_right_wheel_ki_max_sum);
      }
    #endif
  #endif

  #if ENABLE_EMERGENCY_BREAKING == 1
    AEB_setup();
  #endif
  #if ENABLE_FINISH_LINE_DETECTION == 1
    FLD_setup();
  #endif


  #if ENABLE_SETTINGS_MENU != 0
    LcdMenuSetup(MENU_LEFT_ARROW_BUTTON_PIN, MENU_RIGHT_ARROW_BUTTON_PIN, MENU_INCREMENT_BUTTON_PIN, MENU_DECREMENT_BUTTON_PIN);
  #endif

  // serial Initialization
  #if ENABLE_SERIAL_PRINT == 1 || ENABLE_WIRELESS_DEBUG == 1
  #if SERIAL_PORT_TYPE_CONFIGURATION == 1
    #ifdef ENABLE_SERIAL_BUFFER
      SERIAL_PORT.addMemoryForRead(RX_BUFFER, RX_BUFFER_SIZE);
      SERIAL_PORT.addMemoryForWrite(TX_BUFFER, TX_BUFFER_SIZE);
    #endif
  #endif
    SERIAL_PORT.begin(SERIAL_PORT_BAUD_RATE);
    while (!SERIAL_PORT){
      delay(50);
    }
  #endif

  #if ENABLE_WIRELESS_DEBUG == 1
    serial2WifiConnect(SERIAL_PORT, String(DEBUG_WIFI_INIT_SEQUENCE), String(DEBUG_WIFI_SSID), String(DEBUG_WIFI_PASSWORD), String(DEBUG_HOST_IPADDRESS), DEBUG_HOST_PORT);
  #endif

  pixyResult = -1;
  while (pixyResult != PIXY_RESULT_OK)
  {
    pixyResult = g_pixy_1.init();
    #if ENABLE_SERIAL_PRINT == 1
      SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("g_pixy_1.init() = ") + String(pixyResult));
    #endif
    delay(100);
  }
  
  #if CAMERA_ILLUMINATION_LIGHT != 0
    g_pixy_1.setLamp(1,1);
  #else
    g_pixy_1.setLamp(0,0);
  #endif
    
  pixyResult = -1;
  while (pixyResult != PIXY_RESULT_OK)
  {
    pixyResult = g_pixy_1.changeProg("line");
    #if ENABLE_SERIAL_PRINT == 1
      SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("g_pixy_1.changeProg(line) = ") + String(pixyResult));
    #endif
    delay(100);
  }

  g_line_image_frame_width = g_pixy_1.frameWidth;
  g_line_image_frame_height = g_pixy_1.frameHeight;

  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("Setup completed!"));
  #endif
}