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


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void displayParameterValue(String parameter, String value){
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
}


int left_arrow_btn, right_arrow_btn, increment_btn, decrement_btn;

void LcdMenuSetup(int left_arrow, int right_arrow, int up_arrow, int down_arrow){
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
                LCDMENU_MAIN_VIEW,
                LCDMENU_ENABLE_CAR_ENGINE,
                LCDMENU_ENABLE_CAR_STEERING_WHEEL,
                LCDMENU_EMERGENCY_BRAKE_ENABLE_DELAY_S,
                LCDMENU_MIN_XAXIS_ANGLE_VECTOR,
                LCDMENU_MIN_SPEED,
                LCDMENU_MAX_SPEED,
                LCDMENU_MAX_SPEED_CAR_SPEED_KI,
                LCDMENU_MAX_SPEED_CAR_SPEED_KD,
                LCDMENU_CAR_SPEED_KI_MIN_MAX_IMPACT,
		            LCDMENU_MAX_SPEED_AFTER_EMERGENCY_BRAKE_DELAY,
                LCDMENU_ENABLE_FINISH_LINE_DETECTION,
                LCDMENU_FINISH_LINE_ANGLE_TOLERANCE,
                LCDMENU_LOOKAHEAD_MIN_DISTANCE_CM,
                LCDMENU_LOOKAHEAD_MAX_DISTANCE_CM,
                LCDMENU_ENABLE_EMERGENCY_BRAKE,
                LCDMENU_EMERGENCY_BRAKE_DISTANCE_FROM_OBSTACLE_M,
                LCDMENU_ENABLE_DISTANCE_SENSOR1,
                LCDMENU_ENABLE_DISTANCE_SENSOR2,
                LCDMENU_ENABLE_DISTANCE_SENSOR3,
                LCDMENU_EMERGENCY_BREAK_DISTANCE_M,
                LCDMENU_EMERGENCY_BRAKE_MIN_SPEED,
                LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL,
		            LCDMENU_STEERING_WHEEL_ANGLE_OFFSET,
                LCDMENU_CALIBRATION_VIEW_SINGLE_LINE,
                LCDMENU_CALIBRATION_VIEW,
                LCDMENU_LAST_VALUE,
                LCDMENU_ENABLE_REMOTE_START_STOP,
                LCDMENU_BLACK_COLOR_TRESHOLD,
                LCDMENU_ENABLE_PIXY_VECTOR_APPROXIMATION,
                };
  
  
  static int lcdMenuIndex = ((int)LCDMENU_FIRST_VALUE) + 1;
  static int leftArrowButtonState;
  static int rightArrowButtonState;
  static float lcd_print_timeont = 0.0f;
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
    display.clearDisplay();
    switch (lcdMenuIndex) {
      case LCDMENU_STEERING_WHEEL_ANGLE_OFFSET:
        if (incrementButton == HIGH) {
          g_steering_wheel_angle_offset_deg += 0.1f;
        } else if (decrementButton == HIGH) {
          g_steering_wheel_angle_offset_deg -= 0.1f;
        }

        displayParameterValue(String("STR_WHEEL_OFST"), String(g_steering_wheel_angle_offset_deg, 2));
      break;

      case LCDMENU_FINISH_LINE_ANGLE_TOLERANCE:
        if (incrementButton == HIGH) {
          g_finish_line_angle_tolerance += 0.1f;
        } else if (decrementButton == HIGH) {
          g_finish_line_angle_tolerance -= 0.1f;
          g_finish_line_angle_tolerance = MIN(g_finish_line_angle_tolerance, 0.0f);
        }

        displayParameterValue(String("FINISH_LIN_ANG"), String(g_finish_line_angle_tolerance, 2));
      break;

      case LCDMENU_MIN_XAXIS_ANGLE_VECTOR:
        if (incrementButton == HIGH) {
          g_min_x_axis_angle_vector += 0.1f;
        } else if (decrementButton == HIGH) {
          g_min_x_axis_angle_vector -= 0.1f;
        }

        displayParameterValue(String("XAXIS_ANGL_VECT"), String(g_min_x_axis_angle_vector, 2));
      break;

      case LCDMENU_MAIN_VIEW:
        displayParameterValue(String("Loop ms: ")+String(g_loop_time_ms, 2), String("Timer s: ")+String((g_time_passed_ms / 1000.0f), 2));
      break;

      case LCDMENU_ENABLE_CAR_ENGINE:
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
        break;
      
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
      
      case LCDMENU_ENABLE_EMERGENCY_BRAKE:
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
      break;

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

      case LCDMENU_MIN_SPEED:
        if (incrementButton == HIGH) {
          g_min_speed += 0.01f;
        } else if (decrementButton == HIGH) {
          g_min_speed -= 0.01f;
        }
        g_min_speed = MAX(g_min_speed, 0.0f);
        displayParameterValue(String("g_min_speed"), String(g_min_speed, 3));
        break;


      case LCDMENU_MAX_SPEED_CAR_SPEED_KI:
        if (incrementButton == HIGH) {
          g_car_speed_ki += 0.005f;
        } else if (decrementButton == HIGH) {
          g_car_speed_ki -= 0.005f;
        }
        displayParameterValue(String("SPEED_KI"), String(g_car_speed_ki, 4));
      break;

      case LCDMENU_MAX_SPEED_CAR_SPEED_KD:
        if (incrementButton == HIGH) {
          g_car_speed_kd += 0.005f;
        } else if (decrementButton == HIGH) {
          g_car_speed_kd -= 0.005f;
        }

        displayParameterValue(String("SPEED_KD"), String(g_car_speed_kd, 4));
      break;


      case LCDMENU_CAR_SPEED_KI_MIN_MAX_IMPACT:
        if (incrementButton == HIGH) {
          g_car_speed_ki_min_max_impact += 0.1f;
        } else if (decrementButton == HIGH) {
          g_car_speed_ki_min_max_impact -= 0.1f;
        }

        displayParameterValue(String("SPD_KI_MIN_MAX"), String(g_car_speed_ki_min_max_impact, 3));
      break;


      case LCDMENU_MAX_SPEED:
        if (incrementButton == HIGH) {
          g_max_speed += 0.01f;
        } else if (decrementButton == HIGH) {
          g_max_speed -= 0.01f;
        }
        g_max_speed = MAX(g_max_speed, 0.0f);

        displayParameterValue(String("g_max_speed"), String(g_max_speed, 3));
        break;

      case LCDMENU_MAX_SPEED_AFTER_EMERGENCY_BRAKE_DELAY:
        if (incrementButton == HIGH) {
          g_max_speed_after_emergency_brake_delay += 0.5f;
        } else if (decrementButton == HIGH) {
          g_max_speed_after_emergency_brake_delay -= 0.5f;
        }
        g_max_speed_after_emergency_brake_delay = MAX(g_max_speed_after_emergency_brake_delay, 0.0f);

        displayParameterValue(String("MS_AFT_EMBRK_DLY"), String(g_max_speed_after_emergency_brake_delay, 2));
        break;

      case LCDMENU_LOOKAHEAD_MIN_DISTANCE_CM:
        if (incrementButton == HIGH) {
          g_lookahead_min_distance_cm += 0.5f;
        } else if (decrementButton == HIGH) {
          g_lookahead_min_distance_cm -= 0.5f;
        }
        g_lookahead_min_distance_cm = MAX(g_lookahead_min_distance_cm, 0.0f);

        displayParameterValue(String("LOOKAHEAD_MIN"), String(g_lookahead_min_distance_cm, 2));
        break;

      case LCDMENU_LOOKAHEAD_MAX_DISTANCE_CM:
        if (incrementButton == HIGH) {
          g_lookahead_max_distance_cm += 0.5f;
        } else if (decrementButton == HIGH) {
          g_lookahead_max_distance_cm -= 0.5f;
        }
        g_lookahead_max_distance_cm = MAX(g_lookahead_max_distance_cm, 0.0f);

        displayParameterValue(String("LOOKAHEAD_MAX"), String(g_lookahead_max_distance_cm, 2));
        break;
      
      case LCDMENU_EMERGENCY_BREAK_DISTANCE_M:
        if (incrementButton == HIGH) {
          g_emergency_brake_distance_m += 0.005f;
        } else if (decrementButton == HIGH) {
          g_emergency_brake_distance_m -= 0.005f;
        }
        g_emergency_brake_distance_m = MAX(g_emergency_brake_distance_m, 0.0f);

        displayParameterValue(String("EMER_BRK_DIST_M"), String(g_emergency_brake_distance_m, 3));
        break;
      
      case LCDMENU_EMERGENCY_BRAKE_DISTANCE_FROM_OBSTACLE_M:
        if (incrementButton == HIGH) {
          g_emergency_brake_distance_from_obstacle_m += 0.005f;
        } else if (decrementButton == HIGH) {
          g_emergency_brake_distance_from_obstacle_m -= 0.005f;
        }
        g_emergency_brake_distance_from_obstacle_m = MAX(g_emergency_brake_distance_from_obstacle_m, 0.0f);

        displayParameterValue(String("EMR_BR_DIST_OBST"), String(g_emergency_brake_distance_from_obstacle_m, 3));
      break;

      case LCDMENU_EMERGENCY_BRAKE_ENABLE_DELAY_S:
        if (incrementButton == HIGH) {
          g_emergency_brake_enable_delay_s += 0.5f;
        } else if (decrementButton == HIGH) {
          g_emergency_brake_enable_delay_s -= 0.5f;
        }
        g_emergency_brake_enable_delay_s = MAX(g_emergency_brake_enable_delay_s, 0.0f);

        displayParameterValue(String("EMER_BRK_DELY_S"), String(g_emergency_brake_enable_delay_s, 2));
        break;
      
      case LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL:
        if (incrementButton == HIGH) {
          g_lane_width_vector_unit += 0.5f;
        } else if (decrementButton == HIGH) {
          g_lane_width_vector_unit -= 0.5f;
        }
        g_lane_width_vector_unit = MAX(g_lane_width_vector_unit, 0.0f);

        displayParameterValue(String("LANE_W_VECT_UNIT"), String(g_lane_width_vector_unit, 2));
        break;

      case LCDMENU_EMERGENCY_BRAKE_MIN_SPEED:
        if (incrementButton == HIGH) {
          g_emergency_brake_min_speed += 0.01f;
        } else if (decrementButton == HIGH) {
          g_emergency_brake_min_speed -= 0.01f;
        }
        g_emergency_brake_min_speed = MAX(g_emergency_brake_min_speed, 0.0f);

        displayParameterValue(String("EMER_BRK_MIN_SPD"), String(g_emergency_brake_min_speed, 3));
        break;
      
      case LCDMENU_BLACK_COLOR_TRESHOLD:
        if (incrementButton == HIGH) {
          g_black_color_treshold += 0.01f;
        } else if (decrementButton == HIGH) {
          g_black_color_treshold -= 0.01f;
        }
        g_black_color_treshold = MAX(g_black_color_treshold, 0.0f);

        displayParameterValue(String("BLACK_TRESHOLD"), String(g_black_color_treshold, 2));
        break;

      case LCDMENU_CALIBRATION_VIEW:
      {
        LineABC upper_line, lower_line, middle_line;
        IntersectionLines upper_intersection, lower_intersection, left_lane_line_intersection, right_lane_line_intersection;
        float lane_width_;
        
        upper_line = xAxisABC();
        upper_line.C = -IMAGE_MAX_Y;
        lower_line = xAxisABC();
        upper_intersection = intersectionLinesABC(g_middle_lane_line_pixy_1, upper_line);
        lower_intersection = intersectionLinesABC(g_middle_lane_line_pixy_1, lower_line);

        if (upper_intersection.info != 0) {
          displayParameterValue(String("inf"), String("inf"));
        }
        else{
          displayParameterValue(String((upper_intersection.point.x - SCREEN_CENTER_X), 2), String((lower_intersection.point.x - SCREEN_CENTER_X), 2));
        }

        middle_line = xAxisABC();
        middle_line.C = -SCREEN_CENTER_Y;

        left_lane_line_intersection = intersectionLinesABC(g_left_lane_line_pixy_1, middle_line);
        right_lane_line_intersection = intersectionLinesABC(g_right_lane_line_pixy_1, middle_line);
        display.println();
        display.println(String("LaneWdth [vUnit]: "));
        if (left_lane_line_intersection.info == 0 && right_lane_line_intersection.info == 0) {
          lane_width_ = euclidianDistance(left_lane_line_intersection.point, right_lane_line_intersection.point);
          display.println(String(lane_width_, 2));
        }
        else{
          display.println(String("NO_LINE"));
        }
        display.display();
      }
        break;

        case LCDMENU_CALIBRATION_VIEW_SINGLE_LINE:
        {
        LineABC upper_line, lower_line, middle_line, calibration_line;
        IntersectionLines upper_intersection, lower_intersection, left_lane_line_intersection, right_lane_line_intersection;

        middle_line = xAxisABC();
        middle_line.C = -SCREEN_CENTER_Y;

        left_lane_line_intersection = intersectionLinesABC(g_left_lane_line_pixy_1, middle_line);
        right_lane_line_intersection = intersectionLinesABC(g_right_lane_line_pixy_1, middle_line);

        if (left_lane_line_intersection.info == 0 && right_lane_line_intersection.info == 0) {
          if (euclidianDistance(left_lane_line_intersection.point, Point2D{SCREEN_CENTER_X, SCREEN_CENTER_Y}) < euclidianDistance(right_lane_line_intersection.point, Point2D{SCREEN_CENTER_X, SCREEN_CENTER_Y})) {
            calibration_line = g_left_lane_line_pixy_1;
          }
          else{
            calibration_line = g_right_lane_line_pixy_1;
          }
        }
        else if(left_lane_line_intersection.info == 0){
          calibration_line = g_left_lane_line_pixy_1;
        }
        else if(right_lane_line_intersection.info == 0){
          calibration_line = g_right_lane_line_pixy_1;
        }
        else{
          displayParameterValue(String("NO_LINE"), String("NO_LINE"));
          break;
        }


        upper_line = xAxisABC();
        upper_line.C = -IMAGE_MAX_Y;
        lower_line = xAxisABC();
        upper_intersection = intersectionLinesABC(calibration_line, upper_line);
        lower_intersection = intersectionLinesABC(calibration_line, lower_line);

        if (upper_intersection.info != 0) {
          displayParameterValue(String("inf"), String("inf"));
        }
        else{
          displayParameterValue(String((upper_intersection.point.x - SCREEN_CENTER_X), 2), String((lower_intersection.point.x - SCREEN_CENTER_X), 2));
        }
        }
        break;

      default:
        break;
    }
  }
  #if ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE == 1
  }
  #endif
}
