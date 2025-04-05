#include "features/automatic_emergency_braking.h"



void AEB_setup(){
   DistanceSensorsSetupAnalog(
    DISTANCE_SENSOR1_ANALOG_PIN,
    DISTANCE_SENSOR2_ANALOG_PIN,
    DISTANCE_SENSOR3_ANALOG_PIN
    );
   
   pinMode(EMERGENCY_BREAK_LIGHT_PIN, OUTPUT);
   digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, LOW);
}


AEB_out_t automatic_emergency_braking(){
    static MedianFilter AEB_distance_filter(AEB_DISTANCE_FILTER_SIZE);
    float AEB_distance_filtered_m = 0.0f;
    float local_emergency_brake_activation_max_distance_m;
    AEB_out_t out;

    local_emergency_brake_activation_max_distance_m = g_emergency_brake_activation_max_distance_m;
    
    out.active = (int8_t)0;
    out.speed_request_mps = 0.0f;
    out.obstacle_distance_m = 0.0f;
    out.active_loops_count = 0;

    out.obstacle_distance_m = getFrontObstacleDistanceAnalog_m();

    EnableEmergencyBrakeAfterDelay(&g_enable_emergency_brake, g_emergency_brake_enable_delay_s);
    out.enabled = g_enable_emergency_brake;
  
    if (g_enable_emergency_brake != 0 && (g_enable_distance_sensor1 != 0 || g_enable_distance_sensor2 != 0 || g_enable_distance_sensor3 != 0)) {
          }
    else{
      g_emergency_break_active = 0;
      g_emergency_break_loops_count = 0;
      digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, LOW);
      return out;
    }
    if (floatCmp(g_emergency_brake_enable_delay_s, 0.1f) > 0) {
      
      EnableChangeAEBMaxDistanceAfterDelay(&g_enable_change_aeb_max_distance_after_delay_passed, g_enable_change_aeb_max_distance_after_delay_s);
      if (floatCmp(g_enable_change_aeb_max_distance_after_delay_s, 0.01f) < 0) {
        g_enable_change_aeb_max_distance_after_delay_passed = 1;
      }
      
      if (g_enable_change_aeb_max_distance_after_delay_passed) {
        local_emergency_brake_activation_max_distance_m = g_emergency_brake_activation_max_distance_m;
      }
      else{
        local_emergency_brake_activation_max_distance_m = g_emergency_brake_distance_from_obstacle_m;
      }
      
    }
    

    //if (g_emergency_break_active != 0) {
    //  out.obstacle_distance_m = AEB_distance_filter.next(out.obstacle_distance_m);
    //}
    //else{
    //  AEB_distance_filter.clear();
    //}
    
    
    if (out.obstacle_distance_m <= local_emergency_brake_activation_max_distance_m) {
    }
    else{
        g_emergency_break_active = 0;
        g_emergency_break_loops_count = 0;
        digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, LOW);
        return out;
    }

    digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, HIGH);
    g_emergency_break_active = (int8_t)1;
    g_emergency_break_loops_count++;

    

    out.active_loops_count = g_emergency_break_loops_count;
    out.active = g_emergency_break_active;

    if(out.obstacle_distance_m <= g_emergency_brake_distance_from_obstacle_m){
      out.speed_request_mps = (float)STANDSTILL_SPEED;
      #if ENABLE_SERIAL_PRINT == 1
        SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("EMRG_BRK STOP: ") + String(g_emergency_break_loops_count));
      #endif
    }
    else{
      out.speed_request_mps = (float)g_emergency_brake_speed_mps;
      #if ENABLE_SERIAL_PRINT == 1
        SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("EMRG_BRK loop: ") + String(g_emergency_break_loops_count));
      #endif
    }
    return out;
}