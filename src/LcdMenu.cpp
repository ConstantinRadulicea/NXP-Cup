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


#include "LcdMenu.h"

#include "features/automatic_emergency_braking.h"





#if LCD_LIBRARY_ADAFRUIT != 0
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#endif


#if LCD_LIBRARY_SSD1306Ascii != 0
SSD1306AsciiWire display;
#endif



#define ENABLE_LCDMENU_MAIN_VIEW                                  1
#define ENABLE_LCDMENU_ENABLE_CAR_ENGINE                          1
#define ENABLE_LCDMENU_ENABLE_CAR_STEERING_WHEEL                  1
#define ENABLE_LCDMENU_STEERING_WHEEL_ANGLE_OFFSET                1
#define ENABLE_LCDMENU_FRICTION_COEEFFICIENT                      1

#define ENABLE_LCDMENU_MIN_SPEED                                  1
#define ENABLE_LCDMENU_MAX_SPEED                                  1
#define ENABLE_LCDMENU_MAX_SPEED_AFTER_DELAY                      1
#define ENABLE_LCDMENU_ENABLE_MAX_SPEED_AFTER_DELAY               1

#define ENABLE_LCDMENU_LOOKAHEAD_MIN_DISTANCE_CM                  1
#define ENABLE_LCDMENU_LOOKAHEAD_MAX_DISTANCE_CM                  1

#define ENABLE_LCDMENU_ENABLE_FINISH_LINE_DETECTION               1
#define ENABLE_LCDMENU_ENABLE_LINE_DETECTION_AFTER_DELAY          1
#define ENABLE_LCDMENU_FINISH_LINE_ANGLE_TOLERANCE                1

#define ENABLE_LCDMENU_ENABLE_EMERGENCY_BRAKE                     1
#define ENABLE_LCDMENU_ENABLE_EMERGENCY_BRAKE_AFTER_DELAY         1
#define ENABLE_LCDMENU_ENABLE_EMERGENCY_BRAKE_MAX_DISTANCE_AFTER_DELAY         0
#define ENABLE_LCDMENU_EMERGENCY_BRAKE_MIN_SPEED                  1
#define ENABLE_LCDMENU_EMERGENCY_BREAK_DISTANCE_M                 1
#define ENABLE_LCDMENU_EMERGENCY_BRAKE_DISTANCE_FROM_OBSTACLE_M   1

#define ENABLE_LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL                1
#define ENABLE_LCDMENU_MIN_XAXIS_ANGLE_VECTOR                     1

#define ENABLE_LCDMENU_MAX_SPEED_CAR_SPEED_KI                     0
#define ENABLE_LCDMENU_MAX_SPEED_CAR_SPEED_KD                     0
#define ENABLE_LCDMENU_CAR_SPEED_KI_MIN_MAX_IMPACT                0

#define ENABLE_LCDMENU_AUTOMATIC_CALIBRATION_VIEW                 0
#define ENABLE_LCDMENU_CALIBRATION_VIEW_SINGLE_LINE               1
#define ENABLE_LCDMENU_CALIBRATION_VIEW                           1
#define ENABLE_LCDMENU_BIRD_EYE_CALIBRATION_VIEW                  1
#define ENABLE_LCDMENU_CAMERA_OFFSET_Y                            0

#define ENABLE_LCDMENU_ENABLE_DISTANCE_SENSOR1                    0
#define ENABLE_LCDMENU_ENABLE_DISTANCE_SENSOR2                    0
#define ENABLE_LCDMENU_ENABLE_DISTANCE_SENSOR3                    0
#define ENABLE_LCDMENU_ENABLE_REMOTE_START_STOP                   0
#define ENABLE_LCDMENU_BLACK_COLOR_TRESHOLD                       0
#define ENABLE_LCDMENU_ENABLE_PIXY_VECTOR_APPROXIMATION           0

/*
#ifdef TEENSYLC
#define ENABLE_LCDMENU_MAIN_VIEW                                  1
#define ENABLE_LCDMENU_ENABLE_CAR_STEERING_WHEEL                  1
#define ENABLE_LCDMENU_STEERING_WHEEL_ANGLE_OFFSET                1

#define ENABLE_LCDMENU_EMERGENCY_BRAKE_MIN_SPEED                  1
#define ENABLE_LCDMENU_EMERGENCY_BREAK_DISTANCE_M                 1
#define ENABLE_LCDMENU_EMERGENCY_BRAKE_DISTANCE_FROM_OBSTACLE_M   1

#define ENABLE_LCDMENU_CALIBRATION_VIEW                           1
#endif
*/


#if ENABLE_REAR_AXE_STEERING != 0
#define ENABLE_LCDMENU_ENABLE_CAR_STEERING_WHEEL                  0
#define ENABLE_LCDMENU_STEERING_WHEEL_ANGLE_OFFSET                0
#endif

#if ENABLE_SINGLE_AXE_STEERING_NO_RPM != 0
#endif

#if ENABLE_BIRDEYEVIEW == 0
#define ENABLE_LCDMENU_BIRD_EYE_CALIBRATION_VIEW                  0
#endif

#if ENABLE_EDF != 0
  #define ENABLE_LCDMENU_ENABLE_EDF_SPEED                         1
#else
  #define ENABLE_LCDMENU_ENABLE_EDF_SPEED                         0
#endif


void displayParameterValue(String parameter, String value){
  #if LCD_LIBRARY_ADAFRUIT != 0
        display.clearDisplay();
        display.setTextSize(PARAMETER_NAME_TEXT_SIZE);
        display.setTextColor(PARAMETER_NAME_TEXT_COLOR);
        display.setCursor(0, 0);
        display.println(parameter);
        display.setTextSize(PARAMETER_VALUE_TEXT_SIZE);
        display.setTextColor(PARAMETER_VALUE_TEXT_COLOR);
        //display.setCursor(0, 1);
        display.println(value);
        display.display();
  #endif

  #if LCD_LIBRARY_SSD1306Ascii != 0
        display.clear();
        display.setFont(Adafruit5x7);
        display.setCursor(0, 0);
        display.println(parameter);
        display.println(value);
  #endif
}


/*
String FloatToString(float num, int decimals){
    std::ostringstream oss;
    oss.precision(2); // Set precision to 2 decimal places
    oss << std::fixed << num;
    std::string str = oss.str();
    return String(str.c_str());
}
*/
/*
void FloatToStr(float value, char *buffer, int buf_size, int precision) {
    // Use snprintf to safely format the float into the buffer
    // snprintf ensures the buffer size is not exceeded
    snprintf(buffer, buf_size, "%.*f", precision, value);
}

*/


int left_arrow_btn, right_arrow_btn, increment_btn, decrement_btn;


#if LCD_LIBRARY_ADAFRUIT != 0

void LcdDisplaySetup_adafruit(){
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      //Serial.println(F("SSD1306 allocation failed"));
      //for(;;); // Don't proceed, loop forever
    }

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    delay(500);
    // Clear the buffer
    display.clearDisplay();
    display.display();
}
#endif

#if LCD_LIBRARY_SSD1306Ascii != 0
void LcdDisplaySetup_SSD1306Ascii(){
  Wire.begin();
  //Wire.setClock(400000L);

#if RST_PIN >= 0
  display.begin(&Adafruit128x32, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  display.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0
  display.setFont(Adafruit5x7);
}
#endif

void LcdMenuSetup(int left_arrow, int right_arrow, int up_arrow, int down_arrow){
    #if LCD_LIBRARY_ADAFRUIT != 0
      LcdDisplaySetup_adafruit();
    #elif LCD_LIBRARY_SSD1306Ascii != 0
      LcdDisplaySetup_SSD1306Ascii();
    #endif

    left_arrow_btn = left_arrow;
    right_arrow_btn = right_arrow;
    increment_btn = up_arrow;
    decrement_btn = down_arrow;
    pinMode(left_arrow, INPUT_PULLUP);
    pinMode(right_arrow, INPUT_PULLUP);
    pinMode(up_arrow, INPUT_PULLUP);
    pinMode(down_arrow, INPUT_PULLUP);
}

void settingsMenuRoutine() {
  enum LcdMenu {LCDMENU_FIRST_VALUE,
              #if ENABLE_LCDMENU_MAIN_VIEW != 0
                LCDMENU_MAIN_VIEW,
              #endif
              #if ENABLE_LCDMENU_ENABLE_CAR_ENGINE != 0
                LCDMENU_ENABLE_CAR_ENGINE,
              #endif
              #if ENABLE_LCDMENU_ENABLE_CAR_STEERING_WHEEL != 0
                LCDMENU_ENABLE_CAR_STEERING_WHEEL,
              #endif
              #if ENABLE_LCDMENU_STEERING_WHEEL_ANGLE_OFFSET != 0
                LCDMENU_STEERING_WHEEL_ANGLE_OFFSET,
              #endif
              #if ENABLE_LCDMENU_FRICTION_COEEFFICIENT != 0
                LCDMENU_FRICTION_COEEFFICIENT,
              #endif
              #if ENABLE_LCDMENU_MIN_SPEED != 0
                LCDMENU_MIN_SPEED,
              #endif
              #if ENABLE_LCDMENU_MAX_SPEED != 0
                LCDMENU_MAX_SPEED,
              #endif
              #if ENABLE_LCDMENU_MAX_SPEED_AFTER_DELAY != 0
                LCDMENU_MAX_SPEED_AFTER_DELAY,
              #endif
              #if ENABLE_LCDMENU_ENABLE_MAX_SPEED_AFTER_DELAY != 0
                LCDMENU_ENABLE_MAX_SPEED_AFTER_DELAY,
              #endif
              
              #if ENABLE_LCDMENU_LOOKAHEAD_MIN_DISTANCE_CM != 0
                LCDMENU_LOOKAHEAD_MIN_DISTANCE_CM,
              #endif
              #if ENABLE_LCDMENU_LOOKAHEAD_MAX_DISTANCE_CM != 0
                LCDMENU_LOOKAHEAD_MAX_DISTANCE_CM,
              #endif

              #if ENABLE_LCDMENU_ENABLE_FINISH_LINE_DETECTION != 0
                LCDMENU_ENABLE_FINISH_LINE_DETECTION,
              #endif
              #if ENABLE_LCDMENU_ENABLE_LINE_DETECTION_AFTER_DELAY != 0
                LCDMENU_ENABLE_LINE_DETECTION_AFTER_DELAY,
              #endif
              #if ENABLE_LCDMENU_FINISH_LINE_ANGLE_TOLERANCE != 0
                LCDMENU_FINISH_LINE_ANGLE_TOLERANCE,
              #endif

              #if ENABLE_LCDMENU_ENABLE_EMERGENCY_BRAKE != 0
                LCDMENU_ENABLE_EMERGENCY_BRAKE,
              #endif
              #if ENABLE_LCDMENU_ENABLE_EMERGENCY_BRAKE_AFTER_DELAY != 0
                LCDMENU_ENABLE_EMERGENCY_BRAKE_AFTER_DELAY,
              #endif
              #if ENABLE_LCDMENU_ENABLE_EMERGENCY_BRAKE_MAX_DISTANCE_AFTER_DELAY != 0
                LCDMENU_ENABLE_EMERGENCY_BRAKE_MAX_DISTANCE_AFTER_DELAY,
              #endif
              #if ENABLE_LCDMENU_EMERGENCY_BRAKE_MIN_SPEED != 0
                LCDMENU_EMERGENCY_BRAKE_MIN_SPEED,
              #endif
              #if ENABLE_LCDMENU_EMERGENCY_BREAK_DISTANCE_M != 0
                LCDMENU_EMERGENCY_BREAK_DISTANCE_M,
              #endif
              #if ENABLE_LCDMENU_EMERGENCY_BRAKE_DISTANCE_FROM_OBSTACLE_M != 0
                LCDMENU_EMERGENCY_BRAKE_DISTANCE_FROM_OBSTACLE_M,
              #endif

              #if ENABLE_LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL != 0
                LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL,
              #endif
              #if ENABLE_LCDMENU_MIN_XAXIS_ANGLE_VECTOR != 0
		            LCDMENU_MIN_XAXIS_ANGLE_VECTOR,
              #endif

              #if ENABLE_LCDMENU_MAX_SPEED_CAR_SPEED_KI != 0
                LCDMENU_MAX_SPEED_CAR_SPEED_KI,
              #endif
              #if ENABLE_LCDMENU_MAX_SPEED_CAR_SPEED_KD != 0
                LCDMENU_MAX_SPEED_CAR_SPEED_KD,
              #endif
              #if ENABLE_LCDMENU_CAR_SPEED_KI_MIN_MAX_IMPACT != 0
                LCDMENU_CAR_SPEED_KI_MIN_MAX_IMPACT,
              #endif

              #if ENABLE_LCDMENU_CAMERA_OFFSET_Y != 0
                LCDMENU_CAMERA_OFFSET,
              #endif

              
              #if ENABLE_LCDMENU_ENABLE_EDF_SPEED != 0
                LCDMENU_ENABLE_EDF_SPEED,
              #endif
              
              #if ENABLE_LCDMENU_BIRD_EYE_CALIBRATION_VIEW != 0
                LCDMENU_BIRD_EYE_CALIBRATION_VIEW,
              #endif
              #if ENABLE_LCDMENU_AUTOMATIC_CALIBRATION_VIEW != 0
                LCDMENU_AUTOMATIC_CALIBRATION_VIEW,
              #endif
              #if ENABLE_LCDMENU_CALIBRATION_VIEW_SINGLE_LINE != 0
                LCDMENU_CALIBRATION_VIEW_SINGLE_LINE,
              #endif
              #if ENABLE_LCDMENU_CALIBRATION_VIEW != 0
                LCDMENU_CALIBRATION_VIEW,
              #endif
                
                

              #if ENABLE_LCDMENU_ENABLE_DISTANCE_SENSOR1 != 0
                LCDMENU_ENABLE_DISTANCE_SENSOR1,
              #endif
              #if ENABLE_LCDMENU_ENABLE_DISTANCE_SENSOR2 != 0
                LCDMENU_ENABLE_DISTANCE_SENSOR2,
              #endif
              #if ENABLE_LCDMENU_ENABLE_DISTANCE_SENSOR3 != 0
                LCDMENU_ENABLE_DISTANCE_SENSOR3,
              #endif
              #if ENABLE_LCDMENU_ENABLE_REMOTE_START_STOP != 0
                LCDMENU_ENABLE_REMOTE_START_STOP,
              #endif
              #if ENABLE_LCDMENU_BLACK_COLOR_TRESHOLD != 0
                LCDMENU_BLACK_COLOR_TRESHOLD,
              #endif
              #if ENABLE_LCDMENU_ENABLE_PIXY_VECTOR_APPROXIMATION != 0
                LCDMENU_ENABLE_PIXY_VECTOR_APPROXIMATION,
              #endif

                LCDMENU_LAST_VALUE
                };
  
  
  static int lcdMenuIndex = ((int)LCDMENU_FIRST_VALUE) + 1;
  static int leftArrowButtonState;
  static int rightArrowButtonState;
  //static float lcd_print_timeont = 0.0f;
  int incrementButton;
  int decrementButton;
  int leftArrowButtonPrevState, rightArrowButtonPrevState;

  #if ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE == 1
  if (g_enable_car_engine == 0) {
  #endif

  leftArrowButtonPrevState = leftArrowButtonState;
  rightArrowButtonPrevState = rightArrowButtonState;

  leftArrowButtonState = !digitalRead(left_arrow_btn);
  rightArrowButtonState = !digitalRead(right_arrow_btn);
  incrementButton = !digitalRead(increment_btn);
  decrementButton = !digitalRead(decrement_btn);

  //Serial.print(" ");
  //Serial.print(leftArrowButtonState);
  //Serial.print(" ");
  //Serial.print(rightArrowButtonState);
  //Serial.print(" ");
  //Serial.print(incrementButton);
  //Serial.print(" ");
  //Serial.print(decrementButton);
  //Serial.println();

  if (!(leftArrowButtonPrevState == HIGH && leftArrowButtonState == HIGH) && !(rightArrowButtonPrevState == HIGH && rightArrowButtonState == HIGH)) {
    if (rightArrowButtonState == HIGH && lcdMenuIndex >= ((int)LCDMENU_LAST_VALUE) - 1) {
      lcdMenuIndex = ((int)LCDMENU_FIRST_VALUE) + 1;
    } else if (rightArrowButtonState == HIGH) {
      (int)lcdMenuIndex++;
    }

    if (leftArrowButtonState == HIGH && lcdMenuIndex <= ((int)LCDMENU_FIRST_VALUE) + 1) {
      lcdMenuIndex = ((int)LCDMENU_LAST_VALUE) - 1;
    } else if (leftArrowButtonState == HIGH) {
      (int)lcdMenuIndex--;
    }
  }
  ///lcd_print_timeont -= fabsf(g_loop_time_ms);
  if (leftArrowButtonState == HIGH || rightArrowButtonState == HIGH || incrementButton == HIGH || decrementButton == HIGH /*|| lcd_print_timeont <= 0.0f*/) {
    //lcd_print_timeont = 500.0f;

    #if LCD_LIBRARY_ADAFRUIT != 0
      display.clearDisplay();
    #elif LCD_LIBRARY_SSD1306Ascii != 0
      display.clear();
    #endif
    
    switch (lcdMenuIndex) {
    
      #if ENABLE_LCDMENU_STEERING_WHEEL_ANGLE_OFFSET != 0
      case LCDMENU_STEERING_WHEEL_ANGLE_OFFSET:
        if (incrementButton == HIGH) {
          g_steering_wheel_angle_offset_deg += 0.1f;
        } else if (decrementButton == HIGH) {
          g_steering_wheel_angle_offset_deg -= 0.1f;
        }

        displayParameterValue(String("STR_WHEEL_OFFST"), FloatToString(g_steering_wheel_angle_offset_deg, 2));
      break;
    #endif

    #if ENABLE_LCDMENU_FRICTION_COEEFFICIENT != 0
    case LCDMENU_FRICTION_COEEFFICIENT:
      if (incrementButton == HIGH) {
        g_friction_coefficient += 0.01f;
      } else if (decrementButton == HIGH) {
        g_friction_coefficient -= 0.01f;
      }

      g_friction_coefficient = MAX(g_friction_coefficient, 0.0f);

      displayParameterValue(String("g_friction_coefficient"), FloatToString(g_friction_coefficient, 3));
    break;
  #endif

    #if ENABLE_LCDMENU_CAMERA_OFFSET_Y != 0
      case LCDMENU_CAMERA_OFFSET:
        if (incrementButton == HIGH) {
          g_camera_offset_y_m += 0.005f;
        } else if (decrementButton == HIGH) {
          g_camera_offset_y_m -= 0.005f;
        }

        displayParameterValue(String("CAM_OFFST_M"), FloatToString(g_camera_offset_y_m, 3));
      break;
    #endif



    
      
    #if ENABLE_LCDMENU_FINISH_LINE_ANGLE_TOLERANCE != 0
      case LCDMENU_FINISH_LINE_ANGLE_TOLERANCE:
        if (incrementButton == HIGH) {
          g_finish_line_angle_tolerance += 0.1f;
        } else if (decrementButton == HIGH) {
          g_finish_line_angle_tolerance -= 0.1f;
        }

        g_finish_line_angle_tolerance = MAX(g_finish_line_angle_tolerance, 0.0f);

        displayParameterValue(String("FINISH_LIN_ANG"), FloatToString(g_finish_line_angle_tolerance, 2));
      break;
    #endif

    #if ENABLE_LCDMENU_MIN_XAXIS_ANGLE_VECTOR != 0
      case LCDMENU_MIN_XAXIS_ANGLE_VECTOR:
        if (incrementButton == HIGH) {
          g_min_x_axis_angle_vector_deg += 0.1f;
        } else if (decrementButton == HIGH) {
          g_min_x_axis_angle_vector_deg -= 0.1f;
        }

        displayParameterValue(String("XAXIS_ANGL_VECT"), FloatToString(g_min_x_axis_angle_vector_deg, 2));
      break;
    #endif

    #if ENABLE_LCDMENU_MAIN_VIEW != 0
      case LCDMENU_MAIN_VIEW:
      {
        
        float obstacle_distance = getFrontObstacleDistanceAnalog_m();

        #if LCD_LIBRARY_ADAFRUIT != 0
          display.clearDisplay();
          display.setTextSize(PARAMETER_NAME_TEXT_SIZE);
          display.setTextColor(PARAMETER_NAME_TEXT_COLOR);
        #elif LCD_LIBRARY_SSD1306Ascii != 0
        display.clear();
        #endif
        display.setCursor(0, 0);

        display.print("Loop ms: ");
        display.println(FloatToString(g_loop_time_ms, 2));

        display.print("Timer s: ");
        display.println(FloatToString((g_time_passed_ms / 1000.0f), 2));
        display.print("Obst_Dist[m]: ");
        display.println(FloatToString(obstacle_distance, 4));
        display.print("Finish_NOW: ");
        display.println(FloatToString(g_finish_line_detected_now, 0));

  
  
        #if LCD_LIBRARY_ADAFRUIT != 0
          display.display();
        #endif
      
      }
        break;
    #endif

    #if ENABLE_LCDMENU_ENABLE_CAR_ENGINE != 0
      case LCDMENU_ENABLE_CAR_ENGINE:
      {
        if (incrementButton == HIGH) {
          g_emergency_brake_enable_delay_started_count = 0;
          g_emergency_brake_enable_remaining_delay_s = 0.0f;
          g_enable_car_engine = 1;
        }
        else if (decrementButton == HIGH) {
          g_enable_car_engine = 0;
        }
        if (g_enable_car_engine != 0) {
          displayParameterValue(String("ENABLE_ENGINE"), String("Enabled"));
        }
        else{
          displayParameterValue(String("ENABLE_ENGINE"), String("Disabled"));
        }
      }
        break;
      #endif
    
    #if ENABLE_LCDMENU_ENABLE_CAR_STEERING_WHEEL != 0
      case LCDMENU_ENABLE_CAR_STEERING_WHEEL:
        if (incrementButton == HIGH) {
          g_enable_car_steering_wheel = 1;
        }
        else if (decrementButton == HIGH) {
          g_enable_car_steering_wheel = 0;
        }

        if (g_enable_car_steering_wheel != 0) {
          displayParameterValue(String("ENABLE_STEERING"), String("Enabled"));
        }
        else{
          displayParameterValue(String("ENABLE_STEERING"), String("Disabled"));
        }
        break;
      #endif

    #if ENABLE_LCDMENU_ENABLE_EMERGENCY_BRAKE != 0  
      case LCDMENU_ENABLE_EMERGENCY_BRAKE:
      {
        if (incrementButton == HIGH) {
          g_enable_emergency_brake = 1;
        }
        else if (decrementButton == HIGH) {
          g_enable_emergency_brake = 0;
        }

        if (g_enable_emergency_brake != 0) {
          displayParameterValue(String("ENABLE_EMERG_BRK"), String("Enabled"));
        }
        else{
          displayParameterValue(String("ENABLE_EMERG_BRK"), String("Disabled"));
        }
      }
      break;
    #endif

    #if ENABLE_LCDMENU_ENABLE_PIXY_VECTOR_APPROXIMATION != 0
      case LCDMENU_ENABLE_PIXY_VECTOR_APPROXIMATION:
        if (incrementButton == HIGH) {
          g_enable_pixy_vector_approximation = 1;
        }
        else if (decrementButton == HIGH) {
          g_enable_pixy_vector_approximation = 0;
        }

        if (g_enable_pixy_vector_approximation != 0) {
          displayParameterValue(String("ENABLE_VEC_APRX"), String("Enabled"));
        }
        else{
          displayParameterValue(String("ENABLE_VEC_APRX"), String("Disabled"));
        }
      break;
    #endif

    #if ENABLE_LCDMENU_ENABLE_DISTANCE_SENSOR1 != 0
      case LCDMENU_ENABLE_DISTANCE_SENSOR1:
        if (incrementButton == HIGH) {
          g_enable_distance_sensor1 = 1;
        }
        else if (decrementButton == HIGH) {
          g_enable_distance_sensor1 = 0;
        }

        if (g_enable_distance_sensor1 != 0) {
          displayParameterValue(String("ENABLE_DIST_SNS1"), String("Enabled"));
        }
        else{
          displayParameterValue(String("ENABLE_DIST_SNS1"), String("Disabled"));
        }
      break;
    #endif

    #if ENABLE_LCDMENU_ENABLE_DISTANCE_SENSOR2 != 0
      case LCDMENU_ENABLE_DISTANCE_SENSOR2:
        if (incrementButton == HIGH) {
          g_enable_distance_sensor2 = 1;
        }
        else if (decrementButton == HIGH) {
          g_enable_distance_sensor2 = 0;
        }

        if (g_enable_distance_sensor2 != 0) {
          displayParameterValue(String("ENABLE_DIST_SNS2"), String("Enabled"));
        }
        else{
          displayParameterValue(String("ENABLE_DIST_SNS2"), String("Disabled"));
        }
      break;
    #endif

    #if ENABLE_LCDMENU_ENABLE_DISTANCE_SENSOR3 != 0
      case LCDMENU_ENABLE_DISTANCE_SENSOR3:
        if (incrementButton == HIGH) {
          g_enable_distance_sensor3 = 1;
        }
        else if (decrementButton == HIGH) {
          g_enable_distance_sensor3 = 0;
        }

        if (g_enable_distance_sensor3 != 0) {
          displayParameterValue(String("ENABLE_DIST_SNS3"), String("Enabled"));
        }
        else{
          displayParameterValue(String("ENABLE_DIST_SNS3"), String("Disabled"));
        }
      break;
    #endif

    #if ENABLE_LCDMENU_ENABLE_REMOTE_START_STOP != 0
      case LCDMENU_ENABLE_REMOTE_START_STOP:
        if (incrementButton == HIGH) {
          g_enable_remote_start_stop = 1;
        }
        else if (decrementButton == HIGH) {
          g_enable_remote_start_stop = 0;
        }

        if (g_enable_remote_start_stop != 0) {
          displayParameterValue(String("ENABLE_REMOTE"), String("Enabled"));
        }
        else{
          displayParameterValue(String("ENABLE_REMOTE"), String("Disabled"));
        }
      break;
    #endif

    #if ENABLE_LCDMENU_ENABLE_FINISH_LINE_DETECTION != 0
      case LCDMENU_ENABLE_FINISH_LINE_DETECTION:
        if (incrementButton == HIGH) {
          g_enable_finish_line_detection = 1;
        }
        else if (decrementButton == HIGH) {
          g_enable_finish_line_detection = 0;
        }

        if (g_enable_finish_line_detection != 0) {
          displayParameterValue(String("ENABLE_FINSH_LN"), String("Enabled"));
        }
        else{
          displayParameterValue(String("ENABLE_FINSH_LN"), String("Disabled"));
        }
      break;
    #endif

    #if ENABLE_LCDMENU_MIN_SPEED != 0
      case LCDMENU_MIN_SPEED:
        if (incrementButton == HIGH) {
          g_vehicle_min_speed_mps += 0.05f;
        } else if (decrementButton == HIGH) {
          g_vehicle_min_speed_mps -= 0.05f;
        }
        g_vehicle_min_speed_mps = MAX(g_vehicle_min_speed_mps, 0.0f);
        displayParameterValue(String("g_vehicle_min_speed_mps"), FloatToString(g_vehicle_min_speed_mps, 3));
        break;
      #endif

    #if ENABLE_LCDMENU_MAX_SPEED_CAR_SPEED_KI != 0
      case LCDMENU_MAX_SPEED_CAR_SPEED_KI:
        if (incrementButton == HIGH) {
          g_car_speed_mps_ki += 0.005f;
        } else if (decrementButton == HIGH) {
          g_car_speed_mps_ki -= 0.005f;
        }
        displayParameterValue(String("SPEED_KI"), FloatToString(g_car_speed_mps_ki, 4));
      break;
    #endif

    #if ENABLE_LCDMENU_MAX_SPEED_CAR_SPEED_KD != 0
      case LCDMENU_MAX_SPEED_CAR_SPEED_KD:
        if (incrementButton == HIGH) {
          g_car_speed_mps_kd += 0.005f;
        } else if (decrementButton == HIGH) {
          g_car_speed_mps_kd -= 0.005f;
        }
        displayParameterValue(String("SPEED_KD"), FloatToString(g_car_speed_mps_kd, 4));
      break;
    #endif

    #if ENABLE_LCDMENU_CAR_SPEED_KI_MIN_MAX_IMPACT != 0
      case LCDMENU_CAR_SPEED_KI_MIN_MAX_IMPACT:
        if (incrementButton == HIGH) {
          g_car_speed_mps_ki_min_max_impact += 0.1f;
        } else if (decrementButton == HIGH) {
          g_car_speed_mps_ki_min_max_impact -= 0.1f;
        }

        displayParameterValue(String("SPD_KI_MIN_MAX"), FloatToString(g_car_speed_mps_ki_min_max_impact, 3));
      break;
    #endif

    #if ENABLE_LCDMENU_MAX_SPEED != 0
      case LCDMENU_MAX_SPEED:
        if (incrementButton == HIGH) {
          g_vehicle_max_speed_original_mps += 0.05f;
        } else if (decrementButton == HIGH) {
          g_vehicle_max_speed_original_mps -= 0.05f;
        }
        g_vehicle_max_speed_original_mps = MAX(g_vehicle_max_speed_original_mps, 0.0f);

        displayParameterValue(String("g_vehicle_max_speed_original_mps"), FloatToString(g_vehicle_max_speed_original_mps, 3));
        break;
      #endif

    #if ENABLE_LCDMENU_MAX_SPEED_AFTER_DELAY != 0
      case LCDMENU_MAX_SPEED_AFTER_DELAY:
        if (incrementButton == HIGH) {
          g_max_speed_after_delay_mps += 0.05f;
        } else if (decrementButton == HIGH) {
          g_max_speed_after_delay_mps -= 0.05f;
        }
        g_max_speed_after_delay_mps = MAX(g_max_speed_after_delay_mps, 0.0f);
        displayParameterValue(String("g_vehicle_max_speed_after_delay_mps"), FloatToString(g_max_speed_after_delay_mps, 4));
      break;
    #endif

    #if ENABLE_LCDMENU_ENABLE_MAX_SPEED_AFTER_DELAY != 0
      case LCDMENU_ENABLE_MAX_SPEED_AFTER_DELAY:
        if (incrementButton == HIGH) {
          g_max_speed_after_delay_s += 0.5f;
        } else if (decrementButton == HIGH) {
          g_max_speed_after_delay_s -= 0.5f;
        }
        g_max_speed_after_delay_s = MAX(g_max_speed_after_delay_s, 0.0f);

        displayParameterValue(String("MAX_SPEED_DLY_S"), FloatToString(g_max_speed_after_delay_s, 2));
        break;
    #endif

    #if ENABLE_LCDMENU_ENABLE_EMERGENCY_BRAKE_AFTER_DELAY != 0
      case LCDMENU_ENABLE_EMERGENCY_BRAKE_AFTER_DELAY:
        if (incrementButton == HIGH) {
          g_emergency_brake_enable_delay_s += 0.5f;
        } else if (decrementButton == HIGH) {
          g_emergency_brake_enable_delay_s -= 0.5f;
        }
        g_emergency_brake_enable_delay_s = MAX(g_emergency_brake_enable_delay_s, 0.0f);

        displayParameterValue(String("ENABLE_EMER_BRAKE_AFTER_DLY"), FloatToString(g_emergency_brake_enable_delay_s, 2));
        break;
    #endif

    #if ENABLE_LCDMENU_ENABLE_EMERGENCY_BRAKE_MAX_DISTANCE_AFTER_DELAY != 0
      case LCDMENU_ENABLE_EMERGENCY_BRAKE_MAX_DISTANCE_AFTER_DELAY:
        if (incrementButton == HIGH) {
          g_enable_change_aeb_max_distance_after_delay_s += 0.5f;
        } else if (decrementButton == HIGH) {
          g_enable_change_aeb_max_distance_after_delay_s -= 0.5f;
        }
        g_enable_change_aeb_max_distance_after_delay_s = MAX(g_enable_change_aeb_max_distance_after_delay_s, -1.0f);

        displayParameterValue(String("ENABLE_AEB_CHG_MAX_DIST_AFTER_DLY"), FloatToString(g_enable_change_aeb_max_distance_after_delay_s, 2));
        break;
    #endif

    


    #if ENABLE_LCDMENU_ENABLE_LINE_DETECTION_AFTER_DELAY != 0
      case LCDMENU_ENABLE_LINE_DETECTION_AFTER_DELAY:
        if (incrementButton == HIGH) {
          g_enable_finish_line_detection_after_delay_s += 0.5f;
        } else if (decrementButton == HIGH) {
          g_enable_finish_line_detection_after_delay_s -= 0.5f;
        }
        g_enable_finish_line_detection_after_delay_s = MAX(g_enable_finish_line_detection_after_delay_s, 0.0f);

        displayParameterValue(String("ENABLE_LINE_DETECT_AFTER_DLY"), FloatToString(g_enable_finish_line_detection_after_delay_s, 2));
        break;
    #endif

    #if ENABLE_LCDMENU_LOOKAHEAD_MIN_DISTANCE_CM != 0
      case LCDMENU_LOOKAHEAD_MIN_DISTANCE_CM:
        if (incrementButton == HIGH) {
          g_lookahead_min_distance_cm += 0.5f;
        } else if (decrementButton == HIGH) {
          g_lookahead_min_distance_cm -= 0.5f;
        }
        g_lookahead_min_distance_cm = MAX(g_lookahead_min_distance_cm, 0.0f);

        displayParameterValue(String("LOOKAHEAD_MIN"), FloatToString(g_lookahead_min_distance_cm, 2));
        break;
    #endif

    #if ENABLE_LCDMENU_LOOKAHEAD_MAX_DISTANCE_CM != 0
      case LCDMENU_LOOKAHEAD_MAX_DISTANCE_CM:
        if (incrementButton == HIGH) {
          g_lookahead_max_distance_cm += 0.5f;
        } else if (decrementButton == HIGH) {
          g_lookahead_max_distance_cm -= 0.5f;
        }
        g_lookahead_max_distance_cm = MAX(g_lookahead_max_distance_cm, 0.0f);

        displayParameterValue(String("LOOKAHEAD_MAX"), FloatToString(g_lookahead_max_distance_cm, 2));
        break;
    #endif
      
    #if ENABLE_LCDMENU_EMERGENCY_BREAK_DISTANCE_M != 0
      case LCDMENU_EMERGENCY_BREAK_DISTANCE_M:
        if (incrementButton == HIGH) {
          g_emergency_brake_activation_max_distance_m += 0.05f;
        } else if (decrementButton == HIGH) {
          g_emergency_brake_activation_max_distance_m -= 0.05f;
        }
        g_emergency_brake_activation_max_distance_m = MAX(g_emergency_brake_activation_max_distance_m, 0.0f);

        displayParameterValue(String("EMER_BRK_DIST_M"), FloatToString(g_emergency_brake_activation_max_distance_m, 3));
        break;
    #endif
      
    #if ENABLE_LCDMENU_EMERGENCY_BRAKE_DISTANCE_FROM_OBSTACLE_M != 0
      case LCDMENU_EMERGENCY_BRAKE_DISTANCE_FROM_OBSTACLE_M:
        if (incrementButton == HIGH) {
          g_emergency_brake_distance_from_obstacle_m += 0.005f;
        } else if (decrementButton == HIGH) {
          g_emergency_brake_distance_from_obstacle_m -= 0.005f;
        }
        g_emergency_brake_distance_from_obstacle_m = MAX(g_emergency_brake_distance_from_obstacle_m, 0.0f);

        displayParameterValue(String("EMR_BR_DIST_OBST"), FloatToString(g_emergency_brake_distance_from_obstacle_m, 3));
      break;
    #endif
      
    #if ENABLE_LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL != 0
      case LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL:
        if (incrementButton == HIGH) {
          g_lane_width_vector_unit += 0.5f;
        } else if (decrementButton == HIGH) {
          g_lane_width_vector_unit -= 0.5f;
        }
        g_lane_width_vector_unit = MAX(g_lane_width_vector_unit, 0.0f);

        displayParameterValue(String("LANE_W_VECT_UNIT"), FloatToString(g_lane_width_vector_unit, 2));
        break;
    #endif

    #if ENABLE_LCDMENU_EMERGENCY_BRAKE_MIN_SPEED != 0
      case LCDMENU_EMERGENCY_BRAKE_MIN_SPEED:
        if (incrementButton == HIGH) {
          g_emergency_brake_speed_mps += 0.01f;
        } else if (decrementButton == HIGH) {
          g_emergency_brake_speed_mps -= 0.01f;
        }
        g_emergency_brake_speed_mps = MAX(g_emergency_brake_speed_mps, 0.0f);

        displayParameterValue(String("EMER_BRK_MIN_SPD"), FloatToString(g_emergency_brake_speed_mps, 3));
        break;
    #endif
      
    #if ENABLE_LCDMENU_BLACK_COLOR_TRESHOLD != 0
      case LCDMENU_BLACK_COLOR_TRESHOLD:
        if (incrementButton == HIGH) {
          g_black_color_treshold += 0.01f;
        } else if (decrementButton == HIGH) {
          g_black_color_treshold -= 0.01f;
        }
        g_black_color_treshold = MAX(g_black_color_treshold, 0.0f);

        displayParameterValue(String("BLACK_TRESHOLD"), FloatToString(g_black_color_treshold, 2));
        break;
    #endif

        

    #if ENABLE_LCDMENU_CALIBRATION_VIEW != 0
      case LCDMENU_CALIBRATION_VIEW:
      {
        LineABC upper_line, lower_line, middle_line;
        IntersectionLines upper_intersection, lower_intersection, left_lane_line_intersection, right_lane_line_intersection;
        float lane_width_;
        
        upper_line = xAxisABC();
        upper_line.C = -g_line_image_frame_height;
        lower_line = xAxisABC();
        upper_intersection = intersectionLinesABC(g_middle_lane_line_pixy_1, upper_line);
        lower_intersection = intersectionLinesABC(g_middle_lane_line_pixy_1, lower_line);

        if (upper_intersection.info != INTERSECTION_INFO_ONE_INTERSECTION) {
          displayParameterValue(String("inf"), String("inf"));
        }
        else{
          displayParameterValue(FloatToString((upper_intersection.point.x - SCREEN_CENTER_X), 2), FloatToString((lower_intersection.point.x - SCREEN_CENTER_X), 2));
        }

        middle_line = xAxisABC();
        middle_line.C = -SCREEN_CENTER_Y;

        left_lane_line_intersection = intersectionLinesABC(g_left_lane_line_pixy_1, middle_line);
        right_lane_line_intersection = intersectionLinesABC(g_right_lane_line_pixy_1, middle_line);
        display.println();
        display.println(String("LaneWdth [vUnit]: "));
        if (left_lane_line_intersection.info == INTERSECTION_INFO_ONE_INTERSECTION && right_lane_line_intersection.info == INTERSECTION_INFO_ONE_INTERSECTION) {
          lane_width_ = euclidianDistance(left_lane_line_intersection.point, right_lane_line_intersection.point);
          display.println(FloatToString(lane_width_, 2));
        }
        else{
          display.println(String("NO_LINE"));
        }
        #if LCD_LIBRARY_ADAFRUIT != 0
          display.display();
        #endif
      }
        break;
    #endif

    #if ENABLE_LCDMENU_CALIBRATION_VIEW_SINGLE_LINE != 0
        case LCDMENU_CALIBRATION_VIEW_SINGLE_LINE:
        {
        LineABC upper_line, lower_line, calibration_line;
        IntersectionLines upper_intersection, lower_intersection;

        
        calibration_line = closestLineToCurrentTrajectory(g_left_lane_line_pixy_1, g_right_lane_line_pixy_1);
        if (isValidLineABC(calibration_line) == 0) {
          displayParameterValue(String("NO_LINE"), String("NO_LINE"));
          break;
        }

        upper_line = xAxisABC();
        upper_line.C = -g_line_image_frame_height;
        lower_line = xAxisABC();
        upper_intersection = intersectionLinesABC(calibration_line, upper_line);
        lower_intersection = intersectionLinesABC(calibration_line, lower_line);

        if (upper_intersection.info != INTERSECTION_INFO_ONE_INTERSECTION) {
          displayParameterValue(String("inf"), String("inf"));
        }
        else{
          display.println("Single line calibration");
          displayParameterValue(FloatToString((upper_intersection.point.x - SCREEN_CENTER_X), 2), FloatToString((lower_intersection.point.x - SCREEN_CENTER_X), 2));
          //displayParameterValue(FloatToString((upper_intersection.point.x), 2), FloatToString((lower_intersection.point.x), 2));
          display.println("Upper x: " + FloatToString((upper_intersection.point.x), 2));
          display.println("Lower x: " + FloatToString((lower_intersection.point.x), 2));
          display.println("Frame width x: " + FloatToString((g_line_image_frame_width), 2));
        }
        }
        break;
    #endif


    #if ENABLE_LCDMENU_BIRD_EYE_CALIBRATION_VIEW != 0
    case LCDMENU_BIRD_EYE_CALIBRATION_VIEW:{
      LineSegment left_line_segment, right_line_segment;
      enum calibration_state_enum{
        UNCALIBRATE,
        CALIBRATE,
        NO_LINE,
        DISPLAY_ONLY
      };


      calibration_state_enum calibration_state;
      calibration_state = calibration_state_enum::DISPLAY_ONLY;

      if (decrementButton != LOW) {
        calibration_state = calibration_state_enum::UNCALIBRATE;
      }
      else if (incrementButton != LOW) {
        calibration_state = calibration_state_enum::CALIBRATE;
        left_line_segment = g_left_lane_segment;
        right_line_segment = g_right_lane_segment;
        if ((!isValidLineSegment(left_line_segment)) || (!isValidLineSegment(right_line_segment))) {
          calibration_state = calibration_state_enum::NO_LINE;
        }
      }
      

      display.setCursor(0, 0);

      #if LCD_LIBRARY_ADAFRUIT != 0
        display.clearDisplay();
        display.setTextSize(PARAMETER_NAME_TEXT_SIZE);
        display.setTextColor(PARAMETER_NAME_TEXT_COLOR);
      #elif LCD_LIBRARY_SSD1306Ascii != 0
      display.clear();
      #endif

      if (calibration_state == calibration_state_enum::UNCALIBRATE) {
        g_start_line_calibration_acquisition = 0;
        g_birdeye_calibrationdata = BirdEyeCalibrationData{};
        g_line_calibration_data = LineCalibrationData{};
        g_birdeye_calibrationdata.valid = 0;
        display.println("UNCALIBRATE");
      }
      
      
      if (calibration_state == calibration_state_enum::CALIBRATE) {
          if (g_start_line_calibration_acquisition != 0) {
            g_birdeye_calibrationdata = CalculateBirdEyeCalibration_lines(left_line_segment, right_line_segment, g_line_image_frame_width, g_line_image_frame_height, LANE_WIDTH_M);
            if(g_birdeye_calibrationdata.valid){
              g_lane_width_vector_unit = g_birdeye_calibrationdata.src_track_width;
              display.println("CALIBRATE");
            }
            else{
              display.println("NO LINE");
            }
          }
          else{
            g_start_line_calibration_acquisition = 1;
            g_birdeye_calibrationdata.valid = 0;
            display.println("UNCALIBRATE");
          }
      }
      else if (incrementButton == LOW && g_start_line_calibration_acquisition != 0) {
        g_start_line_calibration_acquisition = 0;
      }


      display.print("FL:(");
      display.print(FloatToString(g_birdeye_calibrationdata.birdeye_src_matrix[1].x, 2));
      display.print(";");
      display.print(FloatToString(g_birdeye_calibrationdata.birdeye_src_matrix[1].y, 2));
      display.println(")");

      display.print("RL:(");
      display.print(FloatToString(g_birdeye_calibrationdata.birdeye_src_matrix[0].x, 2));
      display.print(";");
      display.print(FloatToString(g_birdeye_calibrationdata.birdeye_src_matrix[0].y, 2));
      display.println(")");

      display.print("FR:(");
      display.print(FloatToString(g_birdeye_calibrationdata.birdeye_src_matrix[3].x, 2));
      display.print(";");
      display.print(FloatToString(g_birdeye_calibrationdata.birdeye_src_matrix[3].y, 2));
      display.println(")");

      display.print("RR:(");
      display.print(FloatToString(g_birdeye_calibrationdata.birdeye_src_matrix[2].x, 2));
      display.print(";");
      display.print(FloatToString(g_birdeye_calibrationdata.birdeye_src_matrix[2].y, 2));
      display.println(")");


      //left_line_segment = BirdEye_CalibrateLineSegment(g_birdeye_calibrationdata, left_line_segment);
      //right_line_segment = BirdEye_CalibrateLineSegment(g_birdeye_calibrationdata, right_line_segment);

      display.println();

      display.print("Track_w:");
      display.println(FloatToString(g_lane_width_vector_unit, 2));

      #if LCD_LIBRARY_ADAFRUIT != 0
        display.display();
      #endif

      break;
    }
    #endif

    #if ENABLE_LCDMENU_AUTOMATIC_CALIBRATION_VIEW != 0
      case LCDMENU_AUTOMATIC_CALIBRATION_VIEW:{
        LineABC calibration_line;
        enum calibration_state_enum{
          UNCALIBRATE,
          CALIBRATE,
          NO_LINE,
          DISPLAY_ONLY
        };


        calibration_state_enum calibration_state;
        calibration_state = calibration_state_enum::DISPLAY_ONLY;

        if (decrementButton != LOW) {
          calibration_state = calibration_state_enum::UNCALIBRATE;
        }
        else if (incrementButton != LOW) {
          calibration_state = calibration_state_enum::CALIBRATE;
          calibration_line = closestLineToCurrentTrajectory(g_left_lane_line_pixy_1, g_right_lane_line_pixy_1);
          if (isValidLineABC(calibration_line) == 0) {
            calibration_state = calibration_state_enum::NO_LINE;
          }
        }
        

        display.setCursor(0, 0);

        #if LCD_LIBRARY_ADAFRUIT != 0
          display.clearDisplay();
          display.setTextSize(PARAMETER_NAME_TEXT_SIZE);
          display.setTextColor(PARAMETER_NAME_TEXT_COLOR);
        #elif LCD_LIBRARY_SSD1306Ascii != 0
        display.clear();
        #endif

        if (calibration_state == calibration_state_enum::UNCALIBRATE) {
          g_start_line_calibration_acquisition = 0;
          g_line_calibration_data = LineCalibrationData{};
          display.println("UNCALIBRATE");
        }
        
        if (calibration_state == calibration_state_enum::CALIBRATE) {
            if (g_start_line_calibration_acquisition != 0) {
              g_line_calibration_data = lineCalibration(calibration_line);
              display.println("CALIBRATE");
            }
            else{
              g_start_line_calibration_acquisition = 1;
            }
        }
        else if (incrementButton == LOW && g_start_line_calibration_acquisition != 0) {
          g_start_line_calibration_acquisition = 0;
        }


        display.print("Offset[rad]: ");
        display.println(FloatToString(g_line_calibration_data.angle_offset, 5));
        display.println("Rotation point:");
        display.print("(x;y):");
        display.println(String("(") + FloatToString(g_line_calibration_data.rotation_point.x, 2) + String(";") + FloatToString(g_line_calibration_data.rotation_point.y, 2) + String(")"));
        display.print("x offset: ");
        display.println(FloatToString(g_line_calibration_data.x_axis_offset, 5));
        display.print("y offset: ");
        display.println(FloatToString(g_line_calibration_data.y_axis_offset, 5));
        #if LCD_LIBRARY_ADAFRUIT != 0
          display.display();
        #endif

        break;
      }
    #endif


    #if ENABLE_LCDMENU_ENABLE_EDF_SPEED != 0
    case LCDMENU_ENABLE_EDF_SPEED:{
      if (incrementButton == HIGH) {
        g_edf_raw_speed += 5.0f;
      } else if (decrementButton == HIGH) {
        g_edf_raw_speed -= 5.0f;
      }
      g_edf_raw_speed = MAX(g_edf_raw_speed, -1.0f);
      g_edf_raw_speed = MIN(g_edf_raw_speed, 180.0f);

      displayParameterValue(String("EDF_RAW_SPEED"), FloatToString(g_edf_raw_speed, 1));
    }
    break;
    #endif

      default:
        break;
        
    }
  }
  else{
    if(g_start_line_calibration_acquisition != 0){
      g_start_line_calibration_acquisition = 0;
    }
    if(g_start_line_calibration_acquisition_birdeye != 0){
      g_start_line_calibration_acquisition_birdeye = 0;
    }
  }

  
  #if ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE == 1
  }
  #endif
}
