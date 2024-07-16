% Adaptive Kalman Filter for RPM Estimation

% Initialize variables
dt = 1000; % Time step (seconds)
n = 100; % Number of measurements

% Simulate some true RPM data with rapid changes
true_rpm = [100 + 2 * (1:50), 200 + 5 * (51:100)]; % True RPM increases linearly and then rapidly

% Simulate noisy measurements
measurement_noise_std = 5000; % Standard deviation of measurement noise
measurements = true_rpm + measurement_noise_std * randn(size(true_rpm));

measurements = sample;
n = length(measurements);
true_rpm = measurements;

% Kalman Filter initialization
% State vector: [RPM; RPM_rate]
x = [0; 0]; % Initial state estimate (RPM, RPM_rate)
P = eye(2); % Initial estimate error covariance matrix
A = [1 dt; 0 1]; % State transition matrix
H = [1 0]; % Measurement matrix
Q = [1 0; 0 1]; % Initial process noise covariance matrix
R = measurement_noise_std^2; % Measurement noise covariance matrix

% Preallocate arrays for storing the filter results
x_estimates = zeros(2, n); % Estimated states

% Adaptive Kalman Filter loop
for k = 1:n
    % Prediction step
    x = A * x;
    P = A * P * A' + Q;
    
    % Measurement update step
    z = measurements(k); % Current measurement
    y = z - H * x; % Measurement residual
    S = H * P * H' + R; % Residual covariance
    K = P * H' / S; % Kalman gain
    
    % Adaptive adjustment of process noise covariance
    adaptive_factor = min(1, abs(y) / (3 * measurement_noise_std));
    Q = adaptive_factor * [1 0; 0 1];
    
    % Update state estimate if no outlier detected
    x = x + K * y; % Updated state estimate
    P = (eye(2) - K * H) * P; % Updated estimate error covariance
    
    % Store the results
    x_estimates(:, k) = x;
end

% Extract the filtered RPM
filtered_rpm = x_estimates(1, :);

% Plot the results
figure;
plot(1:n, true_rpm, 'g-', 'LineWidth', 2); hold on;
plot(1:n, measurements, 'r.', 'MarkerSize', 10);
plot(1:n, filtered_rpm, 'b-', 'LineWidth', 2);
legend('True RPM', 'Noisy Measurements', 'Filtered RPM (Adaptive Kalman)');
xlabel('Time step');
ylabel('RPM');
title('Adaptive Kalman Filter for RPM Estimation');
grid on;
