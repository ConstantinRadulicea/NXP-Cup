#include "setup.h"
#include "features/imu_data.h"
#include "features/oversteer_mitigation.h"
#include "features/EDF.h"




#if (ENABLE_SERIAL_PRINT == 1 || ENABLE_WIRELESS_DEBUG == 1) && SERIAL_PORT_TYPE_CONFIGURATION == 1 && defined(TEENSY40)
  #define ENABLE_SERIAL_BUFFER
  #define RX_BUFFER_SIZE 4092
  #define TX_BUFFER_SIZE 16384
  static char RX_BUFFER[RX_BUFFER_SIZE];
  static char TX_BUFFER[TX_BUFFER_SIZE];
#endif



void FailureModeMessage(Pixy2 *pixy, float time_passed, String errorText){
  static int8_t engine_was_enable_flag = 0;
  #if ENABLE_SERIAL_PRINT != 0 || ENABLE_SERIAL_PRINT_LIMITED != 0
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("seconds [") + FloatToString(time_passed, 2) + String("] ERROR: " + errorText));
  #endif
  if (time_passed >= CAMERA_ERROR_TIMEOUT_S){
    //g_car_speed_mps = (float)STANDSTILL_SPEED;
    if (pixy->init() < ((int8_t)0))
    {
      if (g_enable_car_engine != 0) {
        engine_was_enable_flag = 1;
      }
      
      g_enable_car_engine = 0;
      #if ENABLE_DRIVERMOTOR == 1
        #if ENABLE_SINGLE_AXE_STEERING_NO_RPM != 0
          g_onemotorpowertrain.SetSpeedRequest_slow(STANDSTILL_SPEED);
        #else
        noInterrupts();
            g_powertrain.SetSpeedRequest_slow(STANDSTILL_SPEED, 0.0, 0, g_max_acceleration, g_max_deceleration);
        interrupts();
        #endif
      #endif
      g_steering_angle_rad = 0.0f;
      #if ENABLE_STEERING_SERVO == 1
        g_steering_wheel.setSteeringWheelAngleDeg(0.0f);
      #endif
    }
    else{
      if (engine_was_enable_flag != 0) {
        engine_was_enable_flag = 0;
        g_enable_car_engine = 1;
      }
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

// 834 <---> 

void setup() {
  int8_t pixyResult;
  //Serial.begin(SERIAL_PORT_BAUD_RATE);
  // Initialization and attachment of the servo and motor
  g_steering_angle_rad = 0.0f;
  #if ENABLE_STEERING_SERVO == 1
    pinMode(STEERING_SERVO_PIN, OUTPUT);
    #if CAR_ID == 1
      g_steering_wheel.steering_servo.attach(STEERING_SERVO_PIN, 500, 2500);
    #elif CAR_ID == 2
    g_steering_wheel.steering_servo.attach(STEERING_SERVO_PIN, 500 + 1000 - 666, 2500 - 1000 + 666);
    //g_steering_wheel.steering_servo.attach(STEERING_SERVO_PIN, 500, 2500);

    #else
      g_steering_wheel.steering_servo.attach(STEERING_SERVO_PIN, 1000, 2000);
    #endif

    g_steering_wheel.SetRawAngleOffset(g_steering_wheel_angle_offset_deg);
    g_steering_wheel.setSteeringWheelAngleDeg(0.0f);
  #endif

  #if ENABLE_DRIVERMOTOR == 1
    #if ENABLE_SINGLE_AXE_STEERING_NO_RPM != 0  
      OneMotorPowerTrainSetup(WHEEL_DIAMETER_M, LEFT_WHEEL_MOTOR_PIN);
    #else
    noInterrupts();
        PowerTrainSetup(WHEEL_DIAMETER_M, TRACK_WIDTH_REAR_WHEELS_M, POWERTRAIN_PID_FREQUENCY_HZ, LEFT_WHEEL_MOTOR_PIN, RIGHT_WHEEL_MOTOR_PIN, RPM_SENSOR_LEFT_WHEEL_PIN, RPM_SENSOR_RIGHT_WHEEL_PIN, OSM_routine);
        g_powertrain.SetLeftWheelPID(g_powertrain_left_wheel_kp, g_powertrain_left_wheel_ki, g_powertrain_left_wheel_kd, g_powertrain_left_wheel_ki_max_sum);
        g_powertrain.SetRightWheelPID(g_powertrain_right_wheel_kp, g_powertrain_right_wheel_ki, g_powertrain_right_wheel_kd, g_powertrain_right_wheel_ki_max_sum);
    interrupts();
    #endif
  #endif

  #if ENABLE_EDF != 0
    EDF_setup(EDF_MOTOR_PIN);
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
  #if SERIAL_PORT_TYPE_CONFIGURATION == 1 && defined(TEENSY40)
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
    //serial2WifiConnectC_str(SERIAL_PORT, DEBUG_WIFI_INIT_SEQUENCE, DEBUG_WIFI_SSID, DEBUG_WIFI_PASSWORD, DEBUG_HOST_IPADDRESS, DEBUG_HOST_PORT);
  #endif

  pixyResult = -1;
  while (pixyResult != PIXY_RESULT_OK)
  {
    pixyResult = g_pixy_1.init();
    #if ENABLE_SERIAL_PRINT != 0 || ENABLE_SERIAL_PRINT_LIMITED != 0
      SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("g_pixy_1.init() = ") + FloatToString(pixyResult, 0));
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
    #if ENABLE_SERIAL_PRINT != 0 || ENABLE_SERIAL_PRINT_LIMITED != 0
      SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("g_pixy_1.changeProg(line) = ") + String(pixyResult));
    #endif
    delay(100);
  }

  g_line_image_frame_width = g_pixy_1.frameWidth;
  g_line_image_frame_height = g_pixy_1.frameHeight;

  #if ENABLE_SERIAL_PRINT != 0 || ENABLE_SERIAL_PRINT_LIMITED != 0
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("Setup completed!"));
  #endif
  #if ENABLE_BIRDEYEVIEW != 0
    initialize_g_birdeye_calibrationdata();
  #endif

  #if ENABLE_IMU != 0
    #if ENABLE_SETTINGS_MENU == 0
      wire.begin();
    #endif
    imu_data_setup();
  #endif
}