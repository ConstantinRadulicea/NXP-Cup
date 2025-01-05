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

#include <iostream>
#include <cmath>
#include "PID.h"

int floatCmp(float num1, float num2) {
    if (fabs(num1 - num2) < FLT_EPSILON) {
        return 0;
    }
    else if (num1 > num2) {
        return 1;
    }
    return -1;
}

void checkResult(const std::string& testName, double expected, double actual) {
    std::cout << testName << ": ";
    if (floatCmp(expected, actual) == 0) {
        std::cout << "PASSED ✅ (Expected: " << expected << ", Got: " << actual << ")\n";
    }
    else {
        std::cout << "FAILED ❌ (Expected: " << expected << ", Got: " << actual << ")\n";
    }
}

void test_proportional_response() {
    PID pid(2.0, 0.0, 0.0, -100.0, 100.0);
    double output = pid.calculate(10, 5);  // Error = 10 - 5 = 5
    checkResult("Proportional Response", 10.0, output);
}

void test_integral_response() {
    PID pid(0.0, 1.0, 0.0, -100.0, 100.0);
    pid.calculate(10, 5, 1.0);  // First call
    double output = pid.calculate(10, 5, 1.0);  // Second call (integral accumulated)
    checkResult("Integral Response", 10.0, output);
}

void test_integral_windup() {
    PID pid(0.0, 1.0, 0.0, -10.0, 10.0, 8.0);  // Integral impact = 8.0
    for (int i = 0; i < 20; i++) {
        pid.calculate(10, 5, 1.0);
    }
    double output = pid.calculate(10, 5, 1.0);
    checkResult("Integral Windup", 8.0, output);
}

void test_derivative_response() {
    PID pid(0.0, 0.0, 1.0, -100.0, 100.0);
    pid.calculate(10, 5, 1.0);
    double output = pid.calculate(10, 7, 1.0);
    checkResult("Derivative Response", -2.0, output);
}

void test_output_saturation() {
    PID pid(1.0, 1.0, 1.0, -10.0, 10.0);
    double output = pid.calculate(100, 0, 1.0);
    checkResult("Output Saturation (Max)", 10.0, output);
    output = pid.calculate(-100, 0, 1.0);
    checkResult("Output Saturation (Min)", -10.0, output);
}

void test_zero_error() {
    PID pid(1.0, 1.0, 1.0, -100.0, 100.0);
    double output = pid.calculate(10, 10);
    checkResult("Zero Error", 0.0, output);
}

void test_zero_Ki() {
    PID pid(1.0, 0.0, 1.0, -100.0, 100.0);
    pid.calculate(10, 5, 1.0);
    double output = pid.calculate(10, 5, 1.0);
    checkResult("Zero Ki", 5.0, output);
}

void test_zero_Kd() {
    PID pid(1.0, 1.0, 0.0, -100.0, 100.0);
    double first_output = pid.calculate(10, 5, 1.0);
    double second_output = pid.calculate(10, 6, 1.0);
    checkResult("Zero Kd", 13.0, second_output);
}

void test_time_step_variation() {
    PID pid(1.0, 1.0, 1.0, -100.0, 100.0);

    double output1 = pid.calculate(10, 5, 0.01);
    double output2 = pid.calculate(10, 5, 10.0);

    std::cout << "Time Step Variation: ";
    if (floatCmp(output1, output2) > 0) {
        std::cout << "PASSED ✅ (Different outputs as expected)\n";
    }
    else {
        std::cout << "FAILED ❌ (Outputs are unexpectedly similar)\n";
    }
}


// Negative Error Handling (should produce a negative proportional response)
void test_negative_error() {
    PID pid(2.0, 0.0, 0.0, -100.0, 100.0);
    double output = pid.calculate(5, 10);  // Error = 5 - 10 = -5
    checkResult("Negative Error", -10.0, output);
}

// Negative Setpoint Handling
void test_negative_setpoint() {
    PID pid(2.0, 0.0, 0.0, -100.0, 100.0);
    double output = pid.calculate(-10, -5);  // Error = -10 - (-5) = -5
    checkResult("Negative Setpoint", -10.0, output);
}

// Negative Kp Test
void test_negative_Kp() {
    PID pid(-2.0, 0.0, 0.0, -100.0, 100.0);
    double output = pid.calculate(10, 5);  // Error = 10 - 5 = 5
    checkResult("Negative Kp", -10.0, output);  // Should invert the normal proportional response
}

// Negative Ki Test (Integral term should counteract accumulated error)
void test_negative_Ki() {
    PID pid(1.0, -1.0, 0.0, -100.0, 100.0);
    pid.calculate(10, 5, 1.0);  // First call
    double output = pid.calculate(10, 5, 1.0);  // Second call (integral should decrease output)
    checkResult("Negative Ki", -5.0, output);  // Ideally cancels out accumulated integral
}

// Negative Kd Test (Derivative term should invert normal behavior)
void test_negative_Kd() {
    PID pid(0.0, 0.0, -1.0, -100.0, 100.0);
    pid.calculate(10, 5, 1.0);
    double output = pid.calculate(10, 7, 1.0);  // Change in error = -2
    checkResult("Negative Kd", 2.0, output);  // Should be inverted
}




// domain testing for the PID class
int main() {
    
    test_proportional_response();
    test_integral_response();
    test_integral_windup();
    test_derivative_response();
    test_output_saturation();
    test_zero_error();
    test_zero_Ki();
    test_zero_Kd();
    test_time_step_variation();
    test_negative_error();
    test_negative_setpoint();
    test_negative_Kp();
    test_negative_Ki();
    test_negative_Kd();

    std::cout << "All tests completed!" << std::endl;
    return 0;
}
