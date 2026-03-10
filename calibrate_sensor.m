%% IMU Calibration
% Load the data
load('Training Data/calib2_straight.mat');

% Extract and flatten the actual data matrices from the Simulink structs
gyro_data = squeeze(out.Sensor_GYRO.signals.values)';
accel_data = squeeze(out.Sensor_ACCEL.signals.values)';

% Define the stationary window. 
% Look at your plot and replace 200 with the correct end index of the flat region!
stationary_start = 1;
stationary_end = 12200; 

% Gyroscope Bias (ISM330DHCX)
gyro_stationary = gyro_data(stationary_start:stationary_end, :);
gyro_bias_x = mean(gyro_stationary(:, 1));
gyro_bias_y = mean(gyro_stationary(:, 2));
gyro_bias_z = mean(gyro_stationary(:, 3));

fprintf('--- Gyroscope Biases (rad/s) ---\n');
fprintf('X: %.6f, Y: %.6f, Z: %.6f\n\n', gyro_bias_x, gyro_bias_y, gyro_bias_z);

% Accelerometer Bias (ISM330DHCX)
accel_stationary = accel_data(stationary_start:stationary_end, :);
accel_bias_x = mean(accel_stationary(:, 1));
accel_bias_y = mean(accel_stationary(:, 2));
% Subtract 9.81 m/s^2 from the Z-axis to account for gravity
accel_bias_z = mean(accel_stationary(:, 3)) - 9.81;

fprintf('--- Accelerometer Biases (m/s^2) ---\n');
fprintf('X: %.6f, Y: %.6f, Z: %.6f\n\n', accel_bias_x, accel_bias_y, accel_bias_z);

% Clear workspace to avoid mixing up datasets
clear out gyro_data accel_data

%% 2. Magnetometer Calibration (Hard and Soft Iron)
% Loading the continuous rotation dataset
load('Training Data/calib1_rotate.mat');

% Extract and flatten the magnetometer data matrix (IIS2MDC)
mag_data = squeeze(out.Sensor_MAG.signals.values)';
mag_x = mag_data(:, 1);
mag_y = mag_data(:, 2);

% Hard Iron Offsets (Finding the center of the ellipse)
mag_max_x = max(mag_x);
mag_min_x = min(mag_x);
mag_max_y = max(mag_y);
mag_min_y = min(mag_y);

hard_iron_x = (mag_max_x + mag_min_x) / 2;
hard_iron_y = (mag_max_y + mag_min_y) / 2;

% Soft Iron Scale Factors (Squishing the ellipse into a circle)
chord_x = (mag_max_x - mag_min_x) / 2;
chord_y = (mag_max_y - mag_min_y) / 2;
avg_radius = (chord_x + chord_y) / 2;

scale_x = avg_radius / chord_x;
scale_y = avg_radius / chord_y;

fprintf('--- Magnetometer Calibration ---\n');
fprintf('Hard Iron Bias (T) - X: %.8e, Y: %.8e\n', hard_iron_x, hard_iron_y);
fprintf('Soft Iron Scale      - X: %.4f, Y: %.4f\n', scale_x, scale_y);