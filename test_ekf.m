% test_ekf.m
clear; clc; close all;

% 1. Load a training dataset
load('Training Data/task1_1 1.mat'); % Change this to test different runs

% 2. Run your filter
[X_Est, P_Est, GT] = myEKF(out);

% 3. Calculate RMSE for X and Y position
% Ensure GT and X_Est are the same length for the calculation
n_samples = min(size(GT, 1), size(X_Est, 1));
err_x = X_Est(1:n_samples, 1) - GT(1:n_samples, 1);
err_y = X_Est(1:n_samples, 2) - GT(1:n_samples, 2);

rmse_x = sqrt(mean(err_x.^2));
rmse_y = sqrt(mean(err_y.^2));
total_rmse = sqrt(rmse_x^2 + rmse_y^2);

fprintf('--- Performance ---\n');
fprintf('RMSE X: %.4f m\n', rmse_x);
fprintf('RMSE Y: %.4f m\n', rmse_y);
fprintf('Total RMSE: %.4f m\n', total_rmse);

% 4. Plot the 2D Trajectory
figure;
plot(GT(:, 1), GT(:, 2), 'k-', 'LineWidth', 2); hold on;
plot(X_Est(:, 1), X_Est(:, 2), 'b--', 'LineWidth', 1.5);
legend('Ground Truth (PhaseSpace)', 'EKF Estimate');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Trajectory: Estimate vs Ground Truth');
grid on; axis equal;