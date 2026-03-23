% test_ekf_modified.m
% Enhanced evaluation script with Heading and Sensor Residual Diagnostics
clear; clc; close all;

%% 1. Load Data
% Ensure the file path matches your local directory structure
load('Training Data/task2_1 1.mat'); 

%% 2. Run Filter
% Calls your EKF function
[X_Est, P_Est, GT] = EKF4(out);

%% 3. Position Performance (RMSE)
% Shift both GT and estimate so that the first GT point is at (0,0) for plotting
origin_xy = GT(1, 1:2);
GT_xy = GT(:, 1:2) - origin_xy;
X_xy = X_Est(:, 1:2) - origin_xy;

n_samples = min(size(GT_xy, 1), size(X_xy, 1));
err_x = X_xy(1:n_samples, 1) - GT_xy(1:n_samples, 1);
err_y = X_xy(1:n_samples, 2) - GT_xy(1:n_samples, 2);

rmse_x = sqrt(mean(err_x.^2));
rmse_y = sqrt(mean(err_y.^2));
total_rmse = sqrt(rmse_x^2 + rmse_y^2);

fprintf('=== Position Performance ===\n');
fprintf('RMSE X: %.4f m\n', rmse_x);
fprintf('RMSE Y: %.4f m\n', rmse_y);
fprintf('Total RMSE (XY): %.4f m\n', total_rmse);

%% 4. Heading (Yaw) Diagnostic 
% Convert Ground Truth Quaternions to Yaw [cite: 50, 52]
quat = squeeze(out.GT_rotation.signals.values); 
if size(quat, 1) == 4, quat = quat.'; end

qw = quat(:, 1); qx = quat(:, 2); qy = quat(:, 3); qz = quat(:, 4);
% Standard Z-axis yaw calculation
yaw_gt = atan2(2*(qw.*qz + qx.*qy), 1 - 2*(qy.^2 + qz.^2));

n_yaw = min(length(yaw_gt), size(X_Est, 1));
yaw_err = wrapToPi(X_Est(1:n_yaw, 3) - yaw_gt(1:n_yaw));
rmse_yaw = sqrt(mean(yaw_err.^2));

fprintf('RMSE Yaw: %.4f rad\n', rmse_yaw);

%% 5. Visualization

% --- Figure 1: 2D Trajectory ---
figure('Name', 'Trajectory Analysis', 'NumberTitle', 'off');
plot(GT_xy(:, 1), GT_xy(:, 2), 'k-', 'LineWidth', 2); hold on;
plot(X_xy(:, 1), X_xy(:, 2), 'b--', 'LineWidth', 1.5);
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Ground Truth (PhaseSpace)', 'EKF Estimate');
title('Robot Trajectory: Estimate vs Ground Truth');
grid on; axis equal;

% --- Figure 2: Heading Comparison ---
figure('Name', 'Heading Diagnostic', 'NumberTitle', 'off');
t_imu = out.Sensor_GYRO.time;
plot(t_imu(1:n_yaw), yaw_gt(1:n_yaw), 'k', 'LineWidth', 2); hold on;
plot(t_imu(1:n_yaw), X_Est(1:n_yaw, 3), 'b--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Yaw Angle (rad)');
legend('GT Yaw', 'EKF Yaw');
title('Heading Diagnostic (Check for Sign/Axis Flips)');
grid on;

% --- Figure 3: Error Over Time ---
figure('Name', 'Position Error', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t_imu(1:n_samples), err_x, 'r');
ylabel('X Error (m)'); title('Position Error Components'); grid on;
subplot(2,1,2);
plot(t_imu(1:n_samples), err_y, 'g');
ylabel('Y Error (m)'); xlabel('Time (s)'); grid on;

%% Helper Functions
function angle_wrapped = wrapToPi(angle)
    angle_wrapped = mod(angle + pi, 2*pi) - pi;
end
% Add this to your test script temporarily, run for task1_1 then task1_2
quat = squeeze(out.GT_rotation.signals.values);
if size(quat,1)==4, quat=quat.'; end
qw=quat(:,1); qx=quat(:,2); qy=quat(:,3); qz=quat(:,4);
yaw_gt = atan2(2*(qw.*qz+qx.*qy), 1-2*(qy.^2+qz.^2));
figure; plot(yaw_gt); title('GT Yaw'); ylabel('rad');
fprintf('Initial GT yaw: %.4f rad\n', yaw_gt(1));
fprintf('GT yaw range: %.4f to %.4f rad\n', min(yaw_gt), max(yaw_gt));