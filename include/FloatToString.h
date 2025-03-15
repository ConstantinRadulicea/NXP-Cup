#ifndef __FLOATTOSTRING_H__
#define __FLOATTOSTRING_H__

#include "Arduino.h"

static void floatToString(float value, char *buffer, int buf_size, int precision) {
    if (buf_size <= 0) return; // Ensure the buffer size is valid
    int index = 0;

    // Handle negative numbers
    if (value < 0) {
        if (index < buf_size - 1) {
            buffer[index++] = '-';
        }
        value = -value; // Make the value positive for further processing
    }

    // Extract integer part
    int integer_part = (int)value;
    float fractional_part = value - integer_part;

    // Convert integer part to string
    int temp = integer_part, digits = 0;
    char int_buffer[20]; // Temporary buffer for integer part
    if (temp == 0) {
        int_buffer[digits++] = '0';
    } else {
        while (temp > 0) {
            int_buffer[digits++] = '0' + (temp % 10);
            temp /= 10;
        }
    }

    // Reverse the integer string into the main buffer
    for (int i = digits - 1; i >= 0 && index < buf_size - 1; i--) {
        buffer[index++] = int_buffer[i];
    }

    // Add the decimal point
    if (precision > 0 && index < buf_size - 1) {
        buffer[index++] = '.';
    }

    // Convert fractional part
    for (int i = 0; i < precision && index < buf_size - 1; i++) {
        fractional_part *= 10;
        int digit = (int)fractional_part;
        buffer[index++] = '0' + digit;
        fractional_part -= digit;
    }

    // Null-terminate the string
    if (index < buf_size) {
        buffer[index] = '\0';
    } else {
        buffer[buf_size - 1] = '\0'; // Ensure null termination if buffer is full
    }
}


static String FloatToString(float num, int decimals){
    char buf[100];
    floatToString(num, buf, 99, decimals);
    return String(buf);
}

static char* FloatToC_str(float num, int decimals){
    char buf[100];
    floatToString(num, buf, 99, decimals);
    return buf;
}


#endif