% Kalman Filter for RPM Estimation

% Initialize variables
dt = 0.01; % Time step (seconds)
n = 100; % Number of measurements

% Simulate some true RPM data (for example purposes)
true_rpm = 100 + 2 * (1:n); % True RPM increases linearly

% Simulate noisy measurements
measurement_noise_std = 5; % Standard deviation of measurement noise
measurements = true_rpm + measurement_noise_std * randn(size(true_rpm));

measurements = sample;
measurement_std = 30000;
n = length(measurements);
% Kalman Filter initialization
% State vector: [RPM; RPM_rate]
x = [0; 0]; % Initial state estimate (RPM, RPM_rate)
P = eye(2); % Initial estimate error covariance matrix
A = [1 dt; 0 1]; % State transition matrix
H = [1 0]; % Measurement matrix
Q = [1 0; 0 1]; % Process noise covariance matrix
R = measurement_std^2; % Measurement noise covariance matrix

% Preallocate arrays for storing the filter results
x_estimates = zeros(2, n); % Estimated states

% Kalman Filter loop
for k = 1:n
    if k == 851
        ff = 1;
    end
    % Prediction step
    x = A * x;
    P = A * P * A' + Q;
    
    % Measurement update step
    z = measurements(k); % Current measurement
    y = z - H * x; % Measurement residual
    S = H * P * H' + R; % Residual covariance
    K = P * H' / S; % Kalman gain
    x = x + K * y; % Updated state estimate
    P = (eye(2) - K * H) * P; % Updated estimate error covariance
    
    % Store the results
    x_estimates(:, k) = x;
end

% Plot the results
figure;
plot(1:n, measurements, 'g-', 'LineWidth', 2); hold on;
plot(1:n, measurements, 'r.', 'MarkerSize', 10);
plot(1:n, x_estimates(1, :), 'b-', 'LineWidth', 2);
legend('True RPM', 'Measurements', 'Kalman Filter Estimate');
xlabel('Time step');
ylabel('RPM');
title('Kalman Filter RPM Estimation');
grid on;