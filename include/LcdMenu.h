#ifndef __LCDMENU_H__
#define __LCDMENU_H__


#if ENABLE_SETTINGS_MENU == 1
void settingsMenuRoutine(LiquidCrystal_I2C &lcd_, int left_arrow_btn, int right_arrow_btn, int increment_btn, int decrement_btn) {
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
                LCDMENU_EMERGENCY_BRAKE_DISTANCE_FROM_OBSTACLE_CM,
                LCDMENU_ENABLE_REMOTE_START_STOP,
                LCDMENU_ENABLE_PIXY_VECTOR_APPROXIMATION,
                LCDMENU_ENABLE_DISTANCE_SENSOR1,
                LCDMENU_ENABLE_DISTANCE_SENSOR2,
                LCDMENU_ENABLE_DISTANCE_SENSOR3,
                LCDMENU_EMERGENCY_BREAK_DISTANCE_CM,
                LCDMENU_EMERGENCY_BRAKE_MIN_SPEED,
                LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL,
                LCDMENU_BLACK_COLOR_TRESHOLD,
		            LCDMENU_STEERING_WHEEL_ANGLE_OFFSET,
                LCDMENU_CALIBRATION_VIEW_SINGLE_LINE,
                LCDMENU_CALIBRATION_VIEW,
                LCDMENU_LAST_VALUE};
  
  
  static int lcdMenuIndex = ((int)LCDMENU_FIRST_VALUE) + 1;
  static int leftArrowButtonState=LOW;
  static int rightArrowButtonState=LOW;
  static float lcd_print_timeont = 0.0f;
  int incrementButton=LOW;
  int decrementButton=LOW;
  int leftArrowButtonPrevState, rightArrowButtonPrevState;

  #if ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE == 1
  if (ENABLE_CAR_ENGINE == 0) {
  #endif

  leftArrowButtonPrevState = leftArrowButtonState;
  rightArrowButtonPrevState = rightArrowButtonState;

  leftArrowButtonState = digitalRead(left_arrow_btn);
  rightArrowButtonState = digitalRead(right_arrow_btn);
  incrementButton = digitalRead(increment_btn);
  decrementButton = digitalRead(decrement_btn);

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
  ///lcd_print_timeont -= fabsf(loop_time_ms);
  if (leftArrowButtonState == HIGH || rightArrowButtonState == HIGH || incrementButton == HIGH || decrementButton == HIGH /*|| lcd_print_timeont <= 0.0f*/) {
    //lcd_print_timeont = 500.0f;
    lcd_.clear();
    switch (lcdMenuIndex) {
      case LCDMENU_STEERING_WHEEL_ANGLE_OFFSET:
        if (incrementButton == HIGH) {
          STEERING_WHEEL_ANGLE_OFFSET += 0.1f;
        } else if (decrementButton == HIGH) {
          STEERING_WHEEL_ANGLE_OFFSET -= 0.1f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("STR_WHEEL_OFST");
        lcd_.setCursor(0, 1);
        lcd_.print(STEERING_WHEEL_ANGLE_OFFSET);
      break;

      case LCDMENU_FINISH_LINE_ANGLE_TOLERANCE:
        if (incrementButton == HIGH) {
          FINISH_LINE_ANGLE_TOLERANCE += 0.1f;
        } else if (decrementButton == HIGH) {
          FINISH_LINE_ANGLE_TOLERANCE -= 0.1f;
          FINISH_LINE_ANGLE_TOLERANCE = MIN(FINISH_LINE_ANGLE_TOLERANCE, 0.0f);
        }
        lcd_.setCursor(0, 0);
        lcd_.print("FINISH_LIN_ANG");
        lcd_.setCursor(0, 1);
        lcd_.print(FINISH_LINE_ANGLE_TOLERANCE);
      break;

      case LCDMENU_MIN_XAXIS_ANGLE_VECTOR:
        if (incrementButton == HIGH) {
          MIN_XAXIS_ANGLE_VECTOR += 0.1f;
        } else if (decrementButton == HIGH) {
          MIN_XAXIS_ANGLE_VECTOR -= 0.1f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("XAXIS_ANGL_VECT");
        lcd_.setCursor(0, 1);
        lcd_.print(MIN_XAXIS_ANGLE_VECTOR);
      break;

      case LCDMENU_MAIN_VIEW:
        lcd_.setCursor(0, 0);
        lcd_.print("Loop ms: ");
        lcd_.print(loop_time_ms);

        lcd_.setCursor(0, 1);
        lcd_.print("Timer s: ");
        lcd_.print((time_passed_ms / 1000.0f));
      break;

      case LCDMENU_ENABLE_CAR_ENGINE:
        if (incrementButton == HIGH) {
          emergency_brake_enable_delay_started_count = 0;
          emergency_brake_enable_remaining_delay_s = 0.0f;
          ENABLE_CAR_ENGINE = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_CAR_ENGINE = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_ENGINE");
        lcd_.setCursor(0, 1);
        if (ENABLE_CAR_ENGINE != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
        break;
      
      case LCDMENU_ENABLE_CAR_STEERING_WHEEL:
        if (incrementButton == HIGH) {
          ENABLE_CAR_STEERING_WHEEL = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_CAR_STEERING_WHEEL = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_STEERING");
        lcd_.setCursor(0, 1);
        if (ENABLE_CAR_STEERING_WHEEL != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
        break;
      
      case LCDMENU_ENABLE_EMERGENCY_BRAKE:
        if (incrementButton == HIGH) {
          ENABLE_EMERGENCY_BRAKE = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_EMERGENCY_BRAKE = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_EMERG_BRK");
        lcd_.setCursor(0, 1);
        if (ENABLE_EMERGENCY_BRAKE != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
      break;

      case LCDMENU_ENABLE_PIXY_VECTOR_APPROXIMATION:
        if (incrementButton == HIGH) {
          ENABLE_PIXY_VECTOR_APPROXIMATION_SOFT = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_PIXY_VECTOR_APPROXIMATION_SOFT = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_VEC_APRX");
        lcd_.setCursor(0, 1);
        if (ENABLE_PIXY_VECTOR_APPROXIMATION_SOFT != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
      break;

      case LCDMENU_ENABLE_DISTANCE_SENSOR1:
        if (incrementButton == HIGH) {
          ENABLE_DISTANCE_SENSOR1_SOFT = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_DISTANCE_SENSOR1_SOFT = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_DIST_SNS1");
        lcd_.setCursor(0, 1);
        if (ENABLE_DISTANCE_SENSOR1_SOFT != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
      break;

      case LCDMENU_ENABLE_DISTANCE_SENSOR2:
        if (incrementButton == HIGH) {
          ENABLE_DISTANCE_SENSOR2_SOFT = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_DISTANCE_SENSOR2_SOFT = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_DIST_SNS2");
        lcd_.setCursor(0, 1);
        if (ENABLE_DISTANCE_SENSOR2_SOFT != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
      break;

      case LCDMENU_ENABLE_DISTANCE_SENSOR3:
        if (incrementButton == HIGH) {
          ENABLE_DISTANCE_SENSOR3_SOFT = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_DISTANCE_SENSOR3_SOFT = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_DIST_SNS3");
        lcd_.setCursor(0, 1);
        if (ENABLE_DISTANCE_SENSOR3_SOFT != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
      break;

      case LCDMENU_ENABLE_REMOTE_START_STOP:
        if (incrementButton == HIGH) {
          ENABLE_REMOTE_START_STOP_SOFT = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_REMOTE_START_STOP_SOFT = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_REMOTE");
        lcd_.setCursor(0, 1);
        if (ENABLE_REMOTE_START_STOP_SOFT != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
      break;

      case LCDMENU_ENABLE_FINISH_LINE_DETECTION:
        if (incrementButton == HIGH) {
          ENABLE_FINISH_LINE_DETECTION_SOFT = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_FINISH_LINE_DETECTION_SOFT = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_FINSH_LN");
        lcd_.setCursor(0, 1);
        if (ENABLE_FINISH_LINE_DETECTION_SOFT != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
      break;

      case LCDMENU_MIN_SPEED:
        if (incrementButton == HIGH) {
          MIN_SPEED += 0.5f;
        } else if (decrementButton == HIGH) {
          MIN_SPEED -= 0.5f;
        }
        MIN_SPEED = MAX(MIN_SPEED, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("MIN_SPEED");
        lcd_.setCursor(0, 1);
        lcd_.print(MIN_SPEED);
        break;


      case LCDMENU_MAX_SPEED_CAR_SPEED_KI:
        if (incrementButton == HIGH) {
          CAR_SPEED_KI += 0.0001f;
        } else if (decrementButton == HIGH) {
          CAR_SPEED_KI -= 0.0001f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("SPEED_KI");
        lcd_.setCursor(0, 1);
        lcd_.print(CAR_SPEED_KI);
      break;

      case LCDMENU_MAX_SPEED_CAR_SPEED_KD:
        if (incrementButton == HIGH) {
          CAR_SPEED_KD += 0.0001f;
        } else if (decrementButton == HIGH) {
          CAR_SPEED_KD -= 0.0001f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("SPEED_KD");
        lcd_.setCursor(0, 1);
        lcd_.print(CAR_SPEED_KD);
      break;


      case LCDMENU_CAR_SPEED_KI_MIN_MAX_IMPACT:
        if (incrementButton == HIGH) {
          CAR_SPEED_KI_MIN_MAX_IMPACT += 0.1f;
        } else if (decrementButton == HIGH) {
          CAR_SPEED_KI_MIN_MAX_IMPACT -= 0.1f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("SPD_KI_MIN_MAX");
        lcd_.setCursor(0, 1);
        lcd_.print(CAR_SPEED_KI_MIN_MAX_IMPACT);
      break;


      case LCDMENU_MAX_SPEED:
        if (incrementButton == HIGH) {
          MAX_SPEED += 0.5f;
        } else if (decrementButton == HIGH) {
          MAX_SPEED -= 0.5f;
        }
        MAX_SPEED = MAX(MAX_SPEED, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("MAX_SPEED");
        lcd_.setCursor(0, 1);
        lcd_.print(MAX_SPEED);
        break;

      case LCDMENU_MAX_SPEED_AFTER_EMERGENCY_BRAKE_DELAY:
        if (incrementButton == HIGH) {
          MAX_SPEED_AFTER_EMERGENCY_BRAKE_DELAY += 0.5f;
        } else if (decrementButton == HIGH) {
          MAX_SPEED_AFTER_EMERGENCY_BRAKE_DELAY -= 0.5f;
        }
        MAX_SPEED_AFTER_EMERGENCY_BRAKE_DELAY = MAX(MAX_SPEED_AFTER_EMERGENCY_BRAKE_DELAY, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("MS_AFT_EMBRK_DLY");
        lcd_.setCursor(0, 1);
        lcd_.print(MAX_SPEED_AFTER_EMERGENCY_BRAKE_DELAY);
        break;

      case LCDMENU_LOOKAHEAD_MIN_DISTANCE_CM:
        if (incrementButton == HIGH) {
          LOOKAHEAD_MIN_DISTANCE_CM += 0.5f;
        } else if (decrementButton == HIGH) {
          LOOKAHEAD_MIN_DISTANCE_CM -= 0.5f;
        }
        LOOKAHEAD_MIN_DISTANCE_CM = MAX(LOOKAHEAD_MIN_DISTANCE_CM, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("LOOKAHEAD_MIN");
        lcd_.setCursor(0, 1);
        lcd_.print(LOOKAHEAD_MIN_DISTANCE_CM);
        break;

      case LCDMENU_LOOKAHEAD_MAX_DISTANCE_CM:
        if (incrementButton == HIGH) {
          LOOKAHEAD_MAX_DISTANCE_CM += 0.5f;
        } else if (decrementButton == HIGH) {
          LOOKAHEAD_MAX_DISTANCE_CM -= 0.5f;
        }
        LOOKAHEAD_MAX_DISTANCE_CM = MAX(LOOKAHEAD_MAX_DISTANCE_CM, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("LOOKAHEAD_MAX");
        lcd_.setCursor(0, 1);
        lcd_.print(LOOKAHEAD_MAX_DISTANCE_CM);
        break;
      
      case LCDMENU_EMERGENCY_BREAK_DISTANCE_CM:
        if (incrementButton == HIGH) {
          EMERGENCY_BREAK_DISTANCE_CM += 0.5f;
        } else if (decrementButton == HIGH) {
          EMERGENCY_BREAK_DISTANCE_CM -= 0.5f;
        }
        EMERGENCY_BREAK_DISTANCE_CM = MAX(EMERGENCY_BREAK_DISTANCE_CM, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("EMER_BRK_DIST_CM");
        lcd_.setCursor(0, 1);
        lcd_.print(EMERGENCY_BREAK_DISTANCE_CM);
        break;
      
      case LCDMENU_EMERGENCY_BRAKE_DISTANCE_FROM_OBSTACLE_CM:
        if (incrementButton == HIGH) {
          EMERGENCY_BREAK_MAX_DISTANCE_FROM_OBSTACLE_CM += 0.5f;
        } else if (decrementButton == HIGH) {
          EMERGENCY_BREAK_MAX_DISTANCE_FROM_OBSTACLE_CM -= 0.5f;
        }
        EMERGENCY_BREAK_MAX_DISTANCE_FROM_OBSTACLE_CM = MAX(EMERGENCY_BREAK_MAX_DISTANCE_FROM_OBSTACLE_CM, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("EMR_BR_DIST_OBST");
        lcd_.setCursor(0, 1);
        lcd_.print(EMERGENCY_BREAK_MAX_DISTANCE_FROM_OBSTACLE_CM);
      break;

      case LCDMENU_EMERGENCY_BRAKE_ENABLE_DELAY_S:
        if (incrementButton == HIGH) {
          EMERGENCY_BRAKE_ENABLE_DELAY_S += 0.5f;
        } else if (decrementButton == HIGH) {
          EMERGENCY_BRAKE_ENABLE_DELAY_S -= 0.5f;
        }
        EMERGENCY_BRAKE_ENABLE_DELAY_S = MAX(EMERGENCY_BRAKE_ENABLE_DELAY_S, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("EMER_BRK_DELY_S");
        lcd_.setCursor(0, 1);
        lcd_.print(EMERGENCY_BRAKE_ENABLE_DELAY_S);
        break;
      
      case LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL:
        if (incrementButton == HIGH) {
          LANE_WIDTH_VECTOR_UNIT_REAL += 0.5f;
        } else if (decrementButton == HIGH) {
          LANE_WIDTH_VECTOR_UNIT_REAL -= 0.5f;
        }
        LANE_WIDTH_VECTOR_UNIT_REAL = MAX(LANE_WIDTH_VECTOR_UNIT_REAL, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("LANE_W_VECT_UNIT");
        lcd_.setCursor(0, 1);
        lcd_.print(LANE_WIDTH_VECTOR_UNIT_REAL);
        break;

      case LCDMENU_EMERGENCY_BRAKE_MIN_SPEED:
        if (incrementButton == HIGH) {
          EMERGENCY_BRAKE_MIN_SPEED += 0.5f;
        } else if (decrementButton == HIGH) {
          EMERGENCY_BRAKE_MIN_SPEED -= 0.5f;
        }
        EMERGENCY_BRAKE_MIN_SPEED = MAX(EMERGENCY_BRAKE_MIN_SPEED, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("EMER_BRK_MIN_SPD");
        lcd_.setCursor(0, 1);
        lcd_.print(EMERGENCY_BRAKE_MIN_SPEED);
        break;
      
      case LCDMENU_BLACK_COLOR_TRESHOLD:
        if (incrementButton == HIGH) {
          BLACK_COLOR_TRESHOLD += 0.01f;
        } else if (decrementButton == HIGH) {
          BLACK_COLOR_TRESHOLD -= 0.01f;
        }
        BLACK_COLOR_TRESHOLD = MAX(BLACK_COLOR_TRESHOLD, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("BLACK_TRESHOLD");
        lcd_.setCursor(0, 1);
        lcd_.print(BLACK_COLOR_TRESHOLD);
        break;

      case LCDMENU_CALIBRATION_VIEW:
      {
        LineABC upper_line, lower_line, middle_line;
        IntersectionLines upper_intersection, lower_intersection, left_lane_line_intersection, right_lane_line_intersection;
        float lane_width_;
        
        upper_line = xAxisABC();
        upper_line.C = -IMAGE_MAX_Y;
        lower_line = xAxisABC();
        upper_intersection = intersectionLinesABC(middle_lane_line_pixy_1, upper_line);
        lower_intersection = intersectionLinesABC(middle_lane_line_pixy_1, lower_line);

        if (upper_intersection.info != 0) {
          lcd_.setCursor(0, 0);
          lcd_.print("inf");
          lcd_.setCursor(0, 1);
          lcd_.print("inf");
        }
        else{
          lcd_.setCursor(0, 0);
          lcd_.print(upper_intersection.point.x - SCREEN_CENTER_X, 2);
          lcd_.setCursor(0, 1);
          lcd_.print(lower_intersection.point.x - SCREEN_CENTER_X, 2);
        }

        middle_line = xAxisABC();
        middle_line.C = -SCREEN_CENTER_Y;

        left_lane_line_intersection = intersectionLinesABC(left_lane_line_pixy_1, middle_line);
        right_lane_line_intersection = intersectionLinesABC(right_lane_line_pixy_1, middle_line);

        lcd_.setCursor(8, 0);
        lcd_.print("LaneWdth");
        lcd_.setCursor(8, 1);
        if (left_lane_line_intersection.info == 0 && right_lane_line_intersection.info == 0) {
          lane_width_ = euclidianDistance(left_lane_line_intersection.point, right_lane_line_intersection.point);
          lcd_.print(lane_width_);
        }
        else{
          lcd_.print("NO_LINE");
        }
      }
        break;

        case LCDMENU_CALIBRATION_VIEW_SINGLE_LINE:
        {
        LineABC upper_line, lower_line, middle_line, calibration_line;
        IntersectionLines upper_intersection, lower_intersection, left_lane_line_intersection, right_lane_line_intersection;

        middle_line = xAxisABC();
        middle_line.C = -SCREEN_CENTER_Y;

        left_lane_line_intersection = intersectionLinesABC(left_lane_line_pixy_1, middle_line);
        right_lane_line_intersection = intersectionLinesABC(right_lane_line_pixy_1, middle_line);

        if (left_lane_line_intersection.info == 0 && right_lane_line_intersection.info == 0) {
          if (euclidianDistance(left_lane_line_intersection.point, Point2D{SCREEN_CENTER_X, SCREEN_CENTER_Y}) < euclidianDistance(right_lane_line_intersection.point, Point2D{SCREEN_CENTER_X, SCREEN_CENTER_Y})) {
            calibration_line = left_lane_line_pixy_1;
          }
          else{
            calibration_line = right_lane_line_pixy_1;
          }
        }
        else if(left_lane_line_intersection.info == 0){
          calibration_line = left_lane_line_pixy_1;
        }
        else if(right_lane_line_intersection.info == 0){
          calibration_line = right_lane_line_pixy_1;
        }
        else{
          lcd_.setCursor(0, 0);
          lcd_.print("NO_LINE");
          lcd_.setCursor(0, 1);
          lcd_.print("NO_LINE");
          break;
        }


        upper_line = xAxisABC();
        upper_line.C = -IMAGE_MAX_Y;
        lower_line = xAxisABC();
        upper_intersection = intersectionLinesABC(calibration_line, upper_line);
        lower_intersection = intersectionLinesABC(calibration_line, lower_line);

        if (upper_intersection.info != 0) {
          lcd_.setCursor(0, 0);
          lcd_.print("inf");
          lcd_.setCursor(0, 1);
          lcd_.print("inf");
        }
        else{
          lcd_.setCursor(0, 0);
          lcd_.print(upper_intersection.point.x - SCREEN_CENTER_X, 2);
          lcd_.setCursor(0, 1);
          lcd_.print(lower_intersection.point.x - SCREEN_CENTER_X, 2);
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
#endif


#endif