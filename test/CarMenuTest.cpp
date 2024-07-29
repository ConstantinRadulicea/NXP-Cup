#include <Arduino.h>
#include <LiquidCrystal_I2C.h>


#define ENABLE_SETTINGS_MENU 1


static float g_lane_width_vector_unit = 60.0f;
static float g_lookahead_min_distance_cm = 16.0f;
static float g_lookahead_max_distance_cm = 30.0f;
static float g_emergency_break_distance_cm = 60.0f;
static float g_min_speed = 96.0f;
static float g_max_speed = 112.0f;
static float g_black_color_treshold = 0.2f; // 0=black, 1=white
static float g_car_length_cm = 17.5f;


#define MENU_RIGHT_ARROW_BUTTON_PIN 14
#define MENU_LEFT_ARROW_BUTTON_PIN 15
#define MENU_DECREMENT_BUTTON_PIN 16
#define MENU_INCREMENT_BUTTON_PIN 17

LiquidCrystal_I2C lcd(0x27, 16, 2);




#if ENABLE_SETTINGS_MENU == 1
void settingsMenuRoutine(LiquidCrystal_I2C &lcd_, int left_arrow_btn, int right_arrow_btn, int increment_btn, int decrement_btn) {
  enum LcdMenu {LCDMENU_FIRST_VALUE,
                LCDMENU_MIN_SPEED,
                LCDMENU_MAX_SPEED,
                LCDMENU_LOOKAHEAD_MIN_DISTANCE_CM,
                LCDMENU_LOOKAHEAD_MAX_DISTANCE_CM,
                LCDMENU_EMERGENCY_BREAK_DISTANCE_CM,
                LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL,
                LCDMENU_BLACK_COLOR_TRESHOLD,
                LCDMENU_LAST_VALUE};
  
  static int lcdMenuIndex = ((int)LCDMENU_FIRST_VALUE) + 1;
  static int leftArrowButtonState=LOW;
  static int rightArrowButtonState=LOW;
  int incrementButton=LOW;
  int decrementButton=LOW;
  int leftArrowButtonPrevState, rightArrowButtonPrevState;

  leftArrowButtonPrevState = leftArrowButtonState;
  rightArrowButtonPrevState = rightArrowButtonState;

  leftArrowButtonState = digitalRead(left_arrow_btn);
  rightArrowButtonState = digitalRead(right_arrow_btn);
  incrementButton = digitalRead(increment_btn);
  decrementButton = digitalRead(decrement_btn);

  if (leftArrowButtonPrevState == HIGH && leftArrowButtonState == HIGH) {
    return;
  }

  if (rightArrowButtonPrevState == HIGH && rightArrowButtonState == HIGH) {
    return;
  }


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

  if (leftArrowButtonState == HIGH || rightArrowButtonState == HIGH || incrementButton == HIGH || decrementButton == HIGH) {
    lcd_.clear();
    delay(100);
    switch (lcdMenuIndex) {
      case LCDMENU_MIN_SPEED:
        if (incrementButton == HIGH) {
          g_min_speed += 0.5f;
        } else if (decrementButton == HIGH) {
          g_min_speed -= 0.5f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("g_min_speed: ");
        lcd_.setCursor(0, 1);
        lcd_.print(g_min_speed);
        break;

      case LCDMENU_MAX_SPEED:
        if (incrementButton == HIGH) {
          g_max_speed += 0.5f;
        } else if (decrementButton == HIGH) {
          g_max_speed -= 0.5f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("g_max_speed: ");
        lcd_.setCursor(0, 1);
        lcd_.print(g_max_speed);
        break;

      case LCDMENU_LOOKAHEAD_MIN_DISTANCE_CM:
        if (incrementButton == HIGH) {
          g_lookahead_min_distance_cm += 0.5f;
        } else if (decrementButton == HIGH) {
          g_lookahead_min_distance_cm -= 0.5f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("LOOKAHEAD_MIN: ");
        lcd_.setCursor(0, 1);
        lcd_.print(g_lookahead_min_distance_cm);
        break;

      case LCDMENU_LOOKAHEAD_MAX_DISTANCE_CM:
        if (incrementButton == HIGH) {
          g_lookahead_max_distance_cm += 0.5f;
        } else if (decrementButton == HIGH) {
          g_lookahead_max_distance_cm -= 0.5f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("LOOKAHEAD_MAX: ");
        lcd_.setCursor(0, 1);
        lcd_.print(g_lookahead_max_distance_cm);
        break;
      
      case LCDMENU_EMERGENCY_BREAK_DISTANCE_CM:
        if (incrementButton == HIGH) {
          g_emergency_break_distance_cm += 0.5f;
        } else if (decrementButton == HIGH) {
          g_emergency_break_distance_cm -= 0.5f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("g_emergency_break_distance_cm: ");
        lcd_.setCursor(0, 1);
        lcd_.print(g_emergency_break_distance_cm);
        break;
      
      case LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL:
        if (incrementButton == HIGH) {
          g_lane_width_vector_unit += 0.5f;
        } else if (decrementButton == HIGH) {
          g_lane_width_vector_unit -= 0.5f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("g_lane_width_vector_unit: ");
        lcd_.setCursor(0, 1);
        lcd_.print(g_lane_width_vector_unit);
        break;
      
      case LCDMENU_BLACK_COLOR_TRESHOLD:
        if (incrementButton == HIGH) {
          g_black_color_treshold += 0.01f;
        } else if (decrementButton == HIGH) {
          g_black_color_treshold -= 0.01f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("g_black_color_treshold: ");
        lcd_.setCursor(0, 1);
        lcd_.print(g_black_color_treshold);
        break;
    }
  }
}
#endif


void setup() {
  Serial.begin(9600);

  #if ENABLE_SETTINGS_MENU == 1
    lcd.init();  //display initialization
    lcd.backlight();  // activate the backlight
    pinMode(MENU_LEFT_ARROW_BUTTON_PIN, INPUT);
    pinMode(MENU_RIGHT_ARROW_BUTTON_PIN, INPUT);
    pinMode(MENU_INCREMENT_BUTTON_PIN, INPUT);
    pinMode(MENU_DECREMENT_BUTTON_PIN, INPUT);
  #endif

  // initialize the pushbutton pin as an input:
}
void loop() {
    #if ENABLE_SETTINGS_MENU == 1
      settingsMenuRoutine(lcd, MENU_LEFT_ARROW_BUTTON_PIN, MENU_RIGHT_ARROW_BUTTON_PIN, MENU_INCREMENT_BUTTON_PIN, MENU_DECREMENT_BUTTON_PIN);
    #endif
}