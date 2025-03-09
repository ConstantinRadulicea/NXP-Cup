% n_plots-Bar Linkage Kinematics
clear;
clc;
close all;

% Given parameters


base = 52.3923645;    % Base link length
driver = 24;            % Driver link length
coupler = 44.3040733;    % Coupler link length
follower = 24.5;          % Follower link length
theta1 = deg2rad(-7.016501778106786);  % Angle between base and x-axis [radians]

ideal_base = 52.0;
ideal_driver = 23.1418591;
ideal_coupler = 43.956;
ideal_theta1 = 0;

arm_wheel_angle = 19.167;

% theta1 = 0;
% theta1 : Angle between base and x-axis [radians]
% theta2 : The angle of the driver (crank) ​ relative to the base
% theta3 : The angle of the coupler relative to the base
% theta4 : The angle of the follower relative to the base

% Define crank angle (input angle)
theta2 = linspace(deg2rad(-180),deg2rad(0), 1000);  % Input crank angle range [0, 2*pi]

% Preallocate arrays for theta3 and ideal_theta4_open
ideal_theta4_open = zeros(size(theta2));
ideal_theta4_crossed = zeros(size(theta2));

final_theta2_open_from_open = zeros(size(theta2));
final_theta2_crossed_from_open = zeros(size(theta2));

final_theta2_open_from_crossed = zeros(size(theta2));
final_theta2_crossed_from_crossed = zeros(size(theta2));

for i = 1:length(theta2)

    [ideal_theta4_open(i), ideal_theta4_crossed(i)] = four_bar_linkage_theta2_to_theta4(ideal_base, ideal_driver, ideal_coupler, follower, ideal_theta1, theta2(i));
    [final_theta2_open_from_open(i), final_theta2_crossed_from_open(i)] = four_bar_linkage_theta4_to_theta2(base, driver, coupler, follower, theta1, ideal_theta4_open(i));
    [final_theta2_open_from_crossed(i), final_theta2_crossed_from_crossed(i)] = four_bar_linkage_theta4_to_theta2(base, driver, coupler, follower, theta1, ideal_theta4_crossed(i));

end

% Plot the results
figure;

n_plots = 5;

subplot(n_plots, 1, 1);
plot(rad2deg(theta2), rad2deg(ideal_theta4_open), 'r', 'LineWidth', 1.5);
hold on;
plot(rad2deg(theta2), rad2deg(ideal_theta4_crossed), 'b', 'LineWidth', 1.5);
title('ideal \theta4 open vs \theta2');
xlabel('ideal \theta2 (deg)');
ylabel('ideal \theta4 (deg)');
legend('ideal \theta4 open', 'ideal \theta4 cross');
grid on;


subplot(n_plots, 1, 2);
plot(rad2deg(final_theta2_open_from_open), rad2deg(ideal_theta4_open), 'r', 'LineWidth', 1.5);
hold on;
plot(rad2deg(final_theta2_crossed_from_open), rad2deg(ideal_theta4_open), 'b', 'LineWidth', 1.5);
title('\theta2 from \theta4 open');
xlabel('\theta2 (deg)');
ylabel('\theta4 (deg)');
legend('\theta4 open', '\theta4 cross');
grid on;


subplot(n_plots, 1, 3);
plot(rad2deg(final_theta2_open_from_crossed), rad2deg(ideal_theta4_crossed), 'r', 'LineWidth', 1.5);
hold on;
plot(rad2deg(final_theta2_crossed_from_crossed), rad2deg(ideal_theta4_crossed), 'b', 'LineWidth', 1.5);
title('\theta2 from \theta4 crossed');
xlabel('\theta2 (deg)');
ylabel('\theta4 (deg)');
legend('\theta4 open', '\theta4 cross');
grid on;


subplot(n_plots, 1, 4);
% plot(rad2deg(final_theta2_open_from_crossed), rad2deg(ideal_theta4_crossed), 'r', 'LineWidth', 1.5);
hold on;
plot(rad2deg(final_theta2_crossed_from_crossed) + 90, rad2deg(ideal_theta4_crossed) + (arm_wheel_angle) + 90, 'b', 'LineWidth', 1.5);
title('\theta2 from \theta4 crossed');
xlabel('\theta2 (deg)');
ylabel('\theta4 (deg)');
legend('\theta4 open', '\theta4 cross');
grid on;

subplot(n_plots, 1, 5);
% plot(rad2deg(final_theta2_open_from_crossed), rad2deg(ideal_theta4_crossed), 'r', 'LineWidth', 1.5);
hold on;
plot(rad2deg(theta2) , rad2deg(ideal_theta4_crossed) + arm_wheel_angle, 'b', 'LineWidth', 1.5);
title('\theta2 from \theta4 crossed');
xlabel('\theta2 (deg)');
ylabel('\theta4 (deg)');
legend('\theta4 open', '\theta4 cross');
grid on;



% Compute the max and min angles
[theta2_max, theta2_min, theta3_max, theta3_min, theta4_max, theta4_min] = four_bar_linkage_max_angles(base, driver, coupler, follower, theta1);

% Display results in degrees
fprintf('Theta2 max: %.2f deg, Theta2 min: %.2f deg\n', rad2deg(theta2_max), rad2deg(theta2_min));
fprintf('Theta3 max: %.2f deg, Theta3 min: %.2f deg\n', rad2deg(theta3_max), rad2deg(theta3_min));
fprintf('Theta4 max: %.2f deg, Theta4 min: %.2f deg\n', rad2deg(theta4_max), rad2deg(theta4_min));
