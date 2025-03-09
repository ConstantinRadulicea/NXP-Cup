clc; clear; close all;

% Define four-bar linkage dimensions
base = 18;      % Fixed link length
driver = 6.75;     % Input link length
coupler = 18;    % Coupler link length
follower = 11;   % Output link length


% Test with an initial theta2
theta1_radians = deg2rad(90); % % Reference fixed pivot angle
theta2_test = deg2rad(0); % Input angle in radians

% Convert theta2 to theta4
[theta4_open, theta4_crossed] = four_bar_linkage_theta2_to_theta4(base, driver, coupler, follower, theta1_radians, theta2_test);
% Convert theta4 back to theta2
[theta2_recovered_open, theta2_recovered_crossed] = four_bar_linkage_theta4_to_theta2(base, driver, coupler, follower, theta1_radians, theta4_open);


% [theta3, theta4_open, theta4_crossed] = four_bar_linkage_from_theta2(base, driver, coupler, follower, theta1, theta2_test);
% Display results
disp('--- Test Results ---');
disp(['Input Theta2 (deg): ', num2str(rad2deg(theta2_test))]);
disp(['Computed Theta4 Open (deg): ', num2str(rad2deg(theta4_open))]);
disp(['Computed Theta4 Crossed (deg): ', num2str(rad2deg(theta4_crossed))]);
disp(['Recovered Theta2 Open (deg): ', num2str(rad2deg(theta2_recovered_open))]);
disp(['Recovered Theta2 Crossed (deg): ', num2str(rad2deg(theta2_recovered_crossed))]);

% Verify if the recovered theta2 matches the original theta2_test
tolerance = 1e-6; % Small numerical tolerance
if abs(theta2_test - theta2_recovered_open) < tolerance
    disp('Theta2 recovery is accurate for Open configuration.');
else
    disp('Theta2 recovery mismatch for Open configuration.');
end

if abs(theta2_test - theta2_recovered_crossed) < tolerance
    disp('Theta2 recovery is accurate for Crossed configuration.');
else
    disp('Theta2 recovery mismatch for Crossed configuration.');
end



ggg = 180 - (-136.2508) + 90;
ggg2 = mod(ggg + 180, 2*180) - 180;
