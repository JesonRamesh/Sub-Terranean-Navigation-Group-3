% test_ekf.m
% Offline evaluation script for Training data 1 (Task 1).
% Before running: run calibration.m and paste Gyro bias, Accel mean, Mag constants into myEKF.m.
% For axis mapping run accelorometer.m and set IDX_FWD, IDX_LAT, SGN_FWD, SGN_LAT in myEKF.m.
% Tuning phases:
%   - Predict-only:   set ENABLE_MAG_UPDATE = false and all ENABLE_TOF*_UPDATE = false in myEKF.m;
%     trajectory should drift but not explode (if it explodes, recheck axes and biases).
%   - Mag only:       ENABLE_MAG_UPDATE = true, all ENABLE_TOF*_UPDATE = false
%   - Full fusion:    all ENABLE_* = true

clear; clc; close all;

% 1. Load a training dataset (Task 1 straight course)
load('Training Data/task1_1 1.mat'); % Change this to test different runs of Task 1

% 2. Run your filter
[X_Est, P_Est, GT] = myEKF(out);

% 3. Calculate RMSE for X and Y position (relative to start origin)
% Shift both GT and estimate so that the first GT point is at (0,0).
origin_xy = GT(1, 1:2);
GT_xy = GT(:, 1:2) - origin_xy;
X_xy = X_Est(:, 1:2) - origin_xy;

% Ensure GT and X_Est are the same length for the calculation
n_samples = min(size(GT_xy, 1), size(X_xy, 1));
err_x = X_xy(1:n_samples, 1) - GT_xy(1:n_samples, 1);
err_y = X_xy(1:n_samples, 2) - GT_xy(1:n_samples, 2);

rmse_x = sqrt(mean(err_x.^2));
rmse_y = sqrt(mean(err_y.^2));
total_rmse = sqrt(rmse_x^2 + rmse_y^2);

fprintf('--- Task 1 Position Performance ---\n');
fprintf('RMSE X: %.4f m\n', rmse_x);
fprintf('RMSE Y: %.4f m\n', rmse_y);
fprintf('Total RMSE (XY): %.4f m\n', total_rmse);

% 3b. (Optional) Yaw RMSE for magnetometer tuning
if isfield(out, 'GT_rotation')
    quat = squeeze(out.GT_rotation.signals.values); % 4 x N or N x 4
    if size(quat, 1) == 4
        quat = quat.';
    end

    % Convert quaternion to yaw (Z-rotation) in arena frame
    qw = quat(:, 1); qx = quat(:, 2); qy = quat(:, 3); qz = quat(:, 4);
    % Standard yaw-from-quaternion (assuming Z-up convention)
    yaw_gt = atan2(2*(qw.*qz + qx.*qy), 1 - 2*(qy.^2 + qz.^2));

    n_yaw = min(length(yaw_gt), size(X_Est, 1));
    yaw_err = wrapToPi(X_Est(1:n_yaw, 3) - yaw_gt(1:n_yaw));
    rmse_yaw = sqrt(mean(yaw_err.^2));

    fprintf('RMSE Yaw: %.4f rad\n', rmse_yaw);
end

% 4. Plot the 2D Trajectory (start at origin)
figure(1);
subplot(1,2,1);
plot(GT_xy(:, 1), GT_xy(:, 2), 'k-', 'LineWidth', 2); hold on;
plot(X_xy(:, 1), X_xy(:, 2), 'b--', 'LineWidth', 1.5);
legend('Ground Truth (PhaseSpace)', 'EKF Estimate');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Trajectory: Estimate vs Ground Truth (Task 1)');
grid on; axis equal;

% subplot(1,2,2);
% plot(, 'b--', 'LineWidth', 1.5);
% legend('Ground Truth (PhaseSpace)', 'EKF Estimate');
% xlabel('X Position (m)');
% ylabel('Y Position (m)');
% title('Robot Trajectory: Estimate vs Ground Truth (Task 1)');
% grid on; axis equal;