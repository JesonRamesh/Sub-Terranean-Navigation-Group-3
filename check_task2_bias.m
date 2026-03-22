% check_task2_bias.m
% Session 4 diagnostic: inspect Task 2 gyro axis 1 and GT velocity for
% the first 10 seconds to determine whether a stationary window exists and
% to measure the gyro mean over candidate windows (t<1s, t<2s, t<3s, t<5s).
%
% Uses the same field access pattern as myEKF.m (verified against the
% SimulationOutput struct layout).
% Does NOT modify myEKF.m.

clear; clc; close all;

load('Training Data/task2_1 1.mat');

% ---- Gyro yaw axis (column 1) — same access as myEKF.m ---------------
t_gyro   = out.Sensor_GYRO.time;                     % Nx1
gyro_raw = squeeze(out.Sensor_GYRO.signals.values)'; % Nx3
gyro_yaw = gyro_raw(:, 1);                           % verified yaw axis

% ---- GT position → velocity magnitude ---------------------------------
% GT data may have duplicate timestamps (dt=0 from Simulink logging).
% Keep only rows where dt > 0 before computing velocity.
t_gt_raw = squeeze(out.GT_time.signals.values);   % Nx1
t_gt_raw = t_gt_raw - t_gt_raw(1);
pos_raw  = squeeze(out.GT_position.signals.values);
if size(pos_raw, 1) == 3, pos_raw = pos_raw'; end  % ensure Nx3

% Remove duplicate-time rows
dt_all = [1; diff(t_gt_raw)];           % keep first row always
valid  = dt_all > 0;
t_gt   = t_gt_raw(valid);
pos    = pos_raw(valid, :);

dt_gt   = diff(t_gt);
vel_x   = diff(pos(:,1)) ./ dt_gt;
vel_y   = diff(pos(:,2)) ./ dt_gt;
vel_mag = sqrt(vel_x.^2 + vel_y.^2);
t_vel   = t_gt(1:end-1);

% ---- Print mean gyro for candidate stationary windows -----------------
windows = [1.0, 2.0, 3.0, 5.0];
fprintf('=== Task 2 Gyro Axis 1 — Stationary Window Analysis ===\n');
fprintf('Hardcoded stationary bias (calib2_straight): 0.00186 rad/s\n\n');
for w = windows
    mask = t_gyro <= w;
    n    = sum(mask);
    mu   = mean(gyro_yaw(mask));
    sg   = std(gyro_yaw(mask));
    fprintf('t < %.1fs  |  N = %5d  |  mean = %+.6f rad/s  |  std = %.6f rad/s\n', ...
            w, n, mu, sg);
end

fprintf('\n');

% Also print GT velocity over the same windows to confirm stationarity
fprintf('=== GT Velocity Magnitude — confirming robot is stationary ===\n');
for w = windows
    mask = t_vel <= w;
    if sum(mask) == 0
        fprintf('t < %.1fs  |  no GT data\n', w);
    else
        fprintf('t < %.1fs  |  mean |v| = %.4f m/s  |  max |v| = %.4f m/s\n', ...
                w, mean(vel_mag(mask)), max(vel_mag(mask)));
    end
end

% ---- Figure: Gyro yaw + GT velocity for first 10 seconds --------------
t_max_plot = min(10.0, t_gyro(end));

figure(1);
subplot(2,1,1);
mask_g = t_gyro <= t_max_plot;
plot(t_gyro(mask_g), gyro_yaw(mask_g), 'b-', 'LineWidth', 0.8);
hold on;
yline(0.00186, 'r--', 'LineWidth', 1.2, 'DisplayName', 'calib bias 0.00186 rad/s');
yline(0,        'k:',  'LineWidth', 0.8);
legend('show', 'Location', 'northeast');
xlabel('Time (s)'); ylabel('Gyro axis 1 (rad/s)');
title('Task 2 — Gyro Yaw Axis 1 (first 10 s)');
grid on;

subplot(2,1,2);
mask_v = t_vel <= t_max_plot;
plot(t_vel(mask_v), vel_mag(mask_v), 'm-', 'LineWidth', 0.8);
xlabel('Time (s)'); ylabel('|v_{GT}| (m/s)');
title('Task 2 — GT Velocity Magnitude (first 10 s)');
grid on;

sgtitle('Session 4 Diagnostic: stationary window check');
