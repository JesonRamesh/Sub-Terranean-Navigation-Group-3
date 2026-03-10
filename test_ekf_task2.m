% test_ekf_task2.m
% Offline evaluation script for Training data 2 (Task 2 circuit course).
% Uses the same myEKF.m implementation and tuning as Task 1.

clear; clc; close all;

% 1. Load a training dataset (Task 2 circuit course)
% TODO: update the filename below to match the provided Task 2 .mat file.
load('Training Data/task2_1 1.mat'); % <-- adjust to your actual Task 2 file name

% 2. Run your filter
[X_Est, P_Est, GT] = myEKF(out);

% 3. Calculate RMSE for X and Y position
n_samples = min(size(GT, 1), size(X_Est, 1));
err_x = X_Est(1:n_samples, 1) - GT(1:n_samples, 1);
err_y = X_Est(1:n_samples, 2) - GT(1:n_samples, 2);

rmse_x = sqrt(mean(err_x.^2));
rmse_y = sqrt(mean(err_y.^2));
total_rmse = sqrt(rmse_x^2 + rmse_y^2);

fprintf('--- Task 2 Position Performance ---\n');
fprintf('RMSE X: %.4f m\n', rmse_x);
fprintf('RMSE Y: %.4f m\n', rmse_y);
fprintf('Total RMSE (XY): %.4f m\n', total_rmse);

% 3b. (Optional) Yaw RMSE for comparison with Task 1
if isfield(out, 'GT_rotation')
    quat = squeeze(out.GT_rotation.signals.values); % 4 x N or N x 4
    if size(quat, 1) == 4
        quat = quat.';
    end

    qw = quat(:, 1); qx = quat(:, 2); qy = quat(:, 3); qz = quat(:, 4);
    yaw_gt = atan2(2*(qw.*qz + qx.*qy), 1 - 2*(qy.^2 + qz.^2));

    n_yaw = min(length(yaw_gt), size(X_Est, 1));
    yaw_err = wrapToPi(X_Est(1:n_yaw, 3) - yaw_gt(1:n_yaw));
    rmse_yaw = sqrt(mean(yaw_err.^2));

    fprintf('RMSE Yaw: %.4f rad\n', rmse_yaw);
end

% 4. Plot the 2D Trajectory
figure;
plot(GT(:, 1), GT(:, 2), 'k-', 'LineWidth', 2); hold on;
plot(X_Est(:, 1), X_Est(:, 2), 'b--', 'LineWidth', 1.5);
legend('Ground Truth (PhaseSpace)', 'EKF Estimate');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Trajectory: Estimate vs Ground Truth (Task 2)');
grid on; axis equal;

