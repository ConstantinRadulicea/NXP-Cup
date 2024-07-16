% Kalman Filter for RPM with Outlier Detection

% Initialize variables
dt = 1; % Time step (seconds)
n = 100; % Number of measurements

% Simulate some true RPM data (for example purposes)
true_rpm = 100 + 2 * (1:n); % True RPM increases linearly

% Simulate noisy measurements with occasional large anomalies
measurement_noise_std = 5; % Standard deviation of measurement noise
measurements = true_rpm + measurement_noise_std * randn(size(true_rpm));
anomaly_indices = [20, 50, 70]; % Indices of anomalies
measurements(anomaly_indices) = measurements(anomaly_indices) + 50 * randn(size(anomaly_indices));

measurements = sample;
measurement_noise_std = std(measurements);
n = length(measurements);

% Kalman Filter initialization
% State vector: [RPM; RPM_rate]
x = [0; 0]; % Initial state estimate (RPM, RPM_rate)
P = eye(2); % Initial estimate error covariance matrix
A = [1 dt; 0 1]; % State transition matrix
H = [1 0]; % Measurement matrix
Q = [1 0; 0 1]; % Process noise covariance matrix
R = measurement_noise_std^2; % Measurement noise covariance matrix

% Preallocate arrays for storing the filter results
x_estimates = zeros(2, n); % Estimated states

% Outlier detection threshold
outlier_threshold = 30 * measurement_noise_std;

% Kalman Filter loop
for k = 1:n
    % Prediction step
    x = A * x;
    P = A * P * A' + Q;
    
    % Measurement update step
    z = measurements(k); % Current measurement
    y = z - H * x; % Measurement residual
    S = H * P * H' + R; % Residual covariance
    K = P * H' / S; % Kalman gain
    
    % Outlier detection
    if abs(y) <= outlier_threshold
        % Update state estimate if no outlier detected
        x = x + K * y; % Updated state estimate
        P = (eye(2) - K * H) * P; % Updated estimate error covariance
    else
        % Skip update if outlier detected
        fprintf('Outlier detected at step %d, skipping update.\n', k);
    end
    
    % Store the results
    x_estimates(:, k) = x;
end

% Extract the filtered RPM
filtered_rpm = x_estimates(1, :);

% Plot the results
figure;
plot(1:n, measurements, 'g-', 'LineWidth', 2); hold on;
plot(1:n, measurements, 'r.', 'MarkerSize', 10);
plot(1:n, filtered_rpm, 'b-', 'LineWidth', 2);
legend('True RPM', 'Noisy Measurements', 'Filtered RPM (Kalman)');
xlabel('Time step');
ylabel('RPM');
title('Kalman Filter for RPM with Outlier Detection');
grid on;
