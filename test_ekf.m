% test_ekf.m
clear; clc; close all;

%% 1. Load dataset
%load('Training Data/task1_1 1.mat');
load('Training Data/task2_1 1.mat');

%% 2. Run EKF
[X_Est, P_Est] = myEKF(out);

%% 3. Build time-aligned GT
imu_time_vec = out.Sensor_GYRO.time;
gt_time_vec  = out.GT_position.time;
gt_pos_raw   = squeeze(out.GT_position.signals.values);
if size(gt_pos_raw, 1) == 3, gt_pos_raw = gt_pos_raw'; end

GT_x_interp = interp1(gt_time_vec, gt_pos_raw(:,1), ...
                       imu_time_vec, 'linear', 'extrap');
GT_y_interp = interp1(gt_time_vec, gt_pos_raw(:,2), ...
                       imu_time_vec, 'linear', 'extrap');

%% 4. RMSE — relative to starting position
origin_x = GT_x_interp(1);
origin_y = GT_y_interp(1);

GT_x_rel  = GT_x_interp - origin_x;
GT_y_rel  = GT_y_interp - origin_y;
EKF_x_rel = X_Est(:,1)  - origin_x;
EKF_y_rel = X_Est(:,2)  - origin_y;

err_x      = EKF_x_rel - GT_x_rel;
err_y      = EKF_y_rel - GT_y_rel;
rmse_x     = sqrt(mean(err_x.^2));
rmse_y     = sqrt(mean(err_y.^2));
total_rmse = sqrt(rmse_x^2 + rmse_y^2);

fprintf('=== Position RMSE ===\n');
fprintf('RMSE X: %.4f m\n', rmse_x);
fprintf('RMSE Y: %.4f m\n', rmse_y);
fprintf('Total:  %.4f m\n', total_rmse);
real_steps = sum(diff(out.Sensor_GYRO.time) >= (1/104 * 0.75));                                                                                                                   
  fprintf('Real IMU steps (dt >= MIN_IMU_DT): %d / %d\n', real_steps, length(out.Sensor_GYRO.time));

%% 5. Yaw RMSE
if isfield(out, 'GT_rotation')
    quat = squeeze(out.GT_rotation.signals.values);
    if size(quat,1) == 4, quat = quat'; end
    gt_rot_time = out.GT_rotation.time;

    qw = quat(:,1); qx_q = quat(:,2);
    qy_q = quat(:,3); qz_q = quat(:,4);
    yaw_gt_raw = atan2(2*(qw.*qz_q + qx_q.*qy_q), ...
                       1 - 2*(qy_q.^2 + qz_q.^2));

    yaw_gt_interp = interp1(gt_rot_time, yaw_gt_raw, ...
                             imu_time_vec, 'linear', 'extrap');
    yaw_err       = wrapToPi(X_Est(:,3) - yaw_gt_interp);
    rmse_yaw      = sqrt(mean(yaw_err.^2));

    fprintf('\n=== Heading ===\n');
    fprintf('RMSE Yaw:        %.4f rad  (%.2f deg)\n', ...
            rmse_yaw, rmse_yaw*180/pi);
    fprintf('Max yaw error:   %.2f deg\n', max(abs(yaw_err))*180/pi);
    fprintf('Final yaw error: %.2f deg\n', abs(yaw_err(end))*180/pi);
end

%% 6. Plot trajectory
figure(1);
subplot(1,2,1);
plot(GT_x_rel,  GT_y_rel,  'k-',  'LineWidth', 2,   ...
     'DisplayName', 'Ground Truth'); hold on;
plot(EKF_x_rel, EKF_y_rel, 'b--', 'LineWidth', 1.5, ...
     'DisplayName', 'EKF Estimate');
legend('Location','best');
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('Trajectory  (RMSE = %.3f m)', total_rmse));
grid on; axis equal;

subplot(1,2,2);
names  = {'x (m)','y (m)','theta (rad)','vx (m/s)','vy (m/s)'};
colors = {'b','r','g','m','c'};
for i = 1:5
    plot(imu_time_vec, X_Est(:,i), colors{i}, ...
         'DisplayName', names{i}); hold on;
end
xlabel('Time (s)'); ylabel('State value');
title('EKF States Over Time');
legend('Location','best'); grid on;

%% Local helper
function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end