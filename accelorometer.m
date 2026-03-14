%% Accelerometer axis and noise analysis using Calibration data 1 (straight + stationary)
clear; clc; close all;

%% Load Calibration data 1 (straight line)
load('Training Data/calib2_straight.mat');  % provides struct 'out'

accel = squeeze(out.Sensor_ACCEL.signals.values);   
time_a = out.Sensor_ACCEL.time;                     
time_a = time_a(:); % Force into a column vector

% Ensure accel is 3 x N
if size(accel, 1) ~= 3 && size(accel, 2) == 3
    accel = accel';
end

%% Plot raw accelerometer axes over time
figure;
plot(time_a, accel(1, :), 'r'); hold on;
plot(time_a, accel(2, :), 'g');
plot(time_a, accel(3, :), 'b');
legend('Accel X', 'Accel Y', 'Accel Z');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Raw Accelerometer Axes (Calibration data 1)');
grid on;

%% Stationary period analysis (first 60 s assumed stationary)
stat_idx_a = time_a < 60;

accel_mean = mean(accel(:, stat_idx_a), 2);
g_measured = norm(accel_mean);
g_true = 9.80665;
scale_factor = g_true / g_measured;
accel_noise_var = var(accel(:, stat_idx_a), 0, 2);

fprintf('=== Accelerometer stationary statistics (first 60 s) ===\n');
fprintf('Mean accel (X,Y,Z): [%.4f  %.4f  %.4f] m/s^2\n', accel_mean);
fprintf('Measured |g|: %.5f m/s^2, scale factor: %.5f\n', g_measured, scale_factor);
fprintf('Noise variance (X,Y,Z): [%.4e  %.4e  %.4e] (m/s^2)^2\n', accel_noise_var);

%% Compare accelerometer axes with ground-truth forward motion
gt_pos = squeeze(out.GT_position.signals.values);   
gt_time = out.GT_time.signals.values;                              

if size(gt_pos, 1) == 3
    gt_pos = gt_pos.';
end

gt_time = gt_time(:);

% Use XY plane for forward motion
x_gt = gt_pos(:, 1);
y_gt = gt_pos(:, 2);

% Numerical differentiation to get velocity in world frame
dt_gt = diff(gt_time);
dt_gt(dt_gt <= 0) = 0.005; % CRITICAL: Prevent division by zero from duplicate timestamps
dt_gt = [dt_gt; mean(dt_gt)];

vx_world = [0; diff(x_gt) ./ dt_gt(1:end-1)];
vy_world = [0; diff(y_gt) ./ dt_gt(1:end-1)];

% Dominant straight-line direction from start to end of straight segment
dx_total = x_gt(end) - x_gt(1);
dy_total = y_gt(end) - y_gt(1);
dir_world = [dx_total; dy_total];
dir_world = dir_world / norm(dir_world + 1e-9);

v_forward_world = vx_world * dir_world(1) + vy_world * dir_world(2);

% Interpolate forward velocity to accelerometer timestamps
[gt_time_unique, unique_idx] = unique(gt_time);
v_forward_unique = v_forward_world(unique_idx);

v_forward_interp = interp1(gt_time_unique, v_forward_unique, time_a, 'linear', 'extrap');

% Remove mean from both accel and velocity for correlation
accel_demean = accel - mean(accel, 2);
v_forward_demean = v_forward_interp - mean(v_forward_interp);

% CRITICAL FIX: Force everything into 1D column vectors for the corr() function
vf_col = v_forward_demean(:);
ax_col = accel_demean(1, :)';
ay_col = accel_demean(2, :)';
az_col = accel_demean(3, :)';

corr_x = corr(ax_col, vf_col);
corr_y = corr(ay_col, vf_col);
corr_z = corr(az_col, vf_col);

fprintf('Correlation of accel axes with forward velocity:\n');
fprintf('  Corr(accel X, v_forward) = %.4f\n', corr_x);
fprintf('  Corr(accel Y, v_forward) = %.4f\n', corr_y);
fprintf('  Corr(accel Z, v_forward) = %.4f\n', corr_z);

figure;
subplot(4,1,1);
plot(time_a, v_forward_interp, 'k');
ylabel('v_{forward} (m/s)');
title('Ground-truth forward velocity and accelerometer axes');
grid on;

subplot(4,1,2);
plot(time_a, accel_demean(1, :), 'r');
ylabel('Accel X (demeaned)');
grid on;

subplot(4,1,3);
plot(time_a, accel_demean(2, :), 'g');
ylabel('Accel Y (demeaned)');
grid on;

subplot(4,1,4);
plot(time_a, accel_demean(3, :), 'b');
ylabel('Accel Z (demeaned)');
xlabel('Time (s)');
grid on;