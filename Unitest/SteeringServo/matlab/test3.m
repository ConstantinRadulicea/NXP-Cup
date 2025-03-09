% Define link lengths for testing
% Define four-bar linkage dimensions
clear;
clc;
close all;

base = 18;       % Fixed link length
driver = 6.75;   % Input link length
coupler = 18;    % Coupler link length
follower = 11;   % Output link length

% Create a linspace of theta2 values from 0 to 360 degrees
theta1_radians = deg2rad(100);
theta2_degrees = linspace(-(2*180), 2*180, 360); % 360 values between 0 and 360 degrees
theta2_radians = deg2rad(theta2_degrees); % Convert to radians

% Preallocate arrays for theta4 results
theta4_open_results = NaN(1, 360);
theta4_crossed_results = NaN(1, 360);
theta2_back_converted_open_1 = NaN(1, 360);
theta2_back_converted_open_2 = NaN(1, 360);
theta2_back_converted_crossed_2 = NaN(1, 360);
theta2_back_converted_crossed_1 = NaN(1, 360);

% Loop through the theta2 values and calculate theta4
for i = 1:length(theta2_radians)
    % Convert theta2 to theta4 (open and crossed)
    [theta4_open, theta4_crossed] = four_bar_linkage_theta2_to_theta4(base, driver, coupler, follower, theta1_radians, theta2_radians(i));
    
    % Store results for theta4
    theta4_open_results(i) = theta4_open;
    theta4_crossed_results(i) = theta4_crossed;
    
    % Convert theta4 back to theta2 (open and crossed) for verification
    [theta2_back_converted_open_, theta2_back_converted_crossed_] = four_bar_linkage_theta4_to_theta2(base, driver, coupler, follower, theta1_radians, theta4_open);
    theta2_back_converted_open_1(i) = theta2_back_converted_open_;
    theta2_back_converted_crossed_1(i) = theta2_back_converted_crossed_;
    
    [theta2_back_converted_open_, theta2_back_converted_crossed_] = four_bar_linkage_theta4_to_theta2(base, driver, coupler, follower, theta1_radians, theta4_crossed);
    theta2_back_converted_crossed_2(i) = theta2_back_converted_crossed_;
    theta2_back_converted_open_2(i) = theta2_back_converted_open_;
end

% Plot the results
figure;

% Plot theta2 vs. theta4 (open and crossed configurations)
subplot(3,1,1);
plot(theta2_degrees, rad2deg(theta4_open_results), 'r', 'LineWidth', 1.5);
hold on;
plot(theta2_degrees, rad2deg(theta4_crossed_results), 'b', 'LineWidth', 1.5);
xlabel('Theta2 (Degrees)');
ylabel('Theta4 (Degrees)');
title('Conversion from Theta2 to Theta4');
legend('Open Configuration', 'Crossed Configuration');
grid on;

% Plot theta2 back from theta4 open configuration (open and crossed configurations)
subplot(3,1,2);
plot(theta2_degrees, rad2deg(theta2_back_converted_open_1), 'r', 'LineWidth', 1.5);
hold on;
plot(theta2_degrees, rad2deg(theta2_back_converted_crossed_1), 'b', 'LineWidth', 1.5);
xlabel('Theta2 (Degrees)');
ylabel('Theta2 Back-Converted (Degrees)');
title('Back-Conversion from Theta4 to Theta2 open configuration');
legend('Open Configuration', 'Crossed Configuration');
grid on;

subplot(3,1,3);
plot(theta2_degrees, rad2deg(theta2_back_converted_open_2), 'r', 'LineWidth', 1.5);
hold on;
plot(theta2_degrees, rad2deg(theta2_back_converted_crossed_2), 'b', 'LineWidth', 1.5);
xlabel('Theta2 (Degrees)');
ylabel('Theta2 Back-Converted (Degrees)');
title('Back-Conversion from Theta4 to Theta2 crossed configuration');
legend('Open Configuration', 'Crossed Configuration');
grid on;
