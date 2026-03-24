% clear all; clc;
% 
% datasets = {
%     'Training Data/task1_1 1.mat', 'T1_D1';
%     'Training Data/task1_2 1.mat', 'T1_D2';
%     'Training Data/task1_3.mat', 'T1_D3';
%     'Training Data/task1_4.mat', 'T1_D4';
%     'Training Data/task2_1 1.mat', 'T2_D1';
%     'Training Data/task2_2 1.mat', 'T2_D2';
%     'Training Data/task2_3 1.mat', 'T2_D3';
%     'Training Data/task2_4 1.mat', 'T2_D4';
% };
% 
% results = zeros(8, 1);
% 
% for d = 1:8
%     clear myEKF;
%     load(datasets{d, 1});
% 
%     gyro_data  = squeeze(out.Sensor_GYRO.signals.values)';
%     accel_data = squeeze(out.Sensor_ACCEL.signals.values)';
%     mag_data   = squeeze(out.Sensor_MAG.signals.values)';
%     tof1_data  = squeeze(out.Sensor_ToF1.signals.values);
%     tof2_data  = squeeze(out.Sensor_ToF2.signals.values);
%     tof3_data  = squeeze(out.Sensor_ToF3.signals.values);
% 
%     N = size(gyro_data, 1);
%     X_all = zeros(N, 8);
% 
%     for k = 1:N
%         [xe, ~] = myEKF(accel_data(k,:)', ...
%                         gyro_data(k,:)',  ...
%                         mag_data(k,:)',   ...
%                         tof1_data(k,:)', ...
%                         tof2_data(k,:)', ...
%                         tof3_data(k,:)', ...
%                         0, 0);
%         X_all(k,:) = xe';
%     end
% 
%     imu_time = out.Sensor_GYRO.time;
%     gt_pos   = squeeze(out.GT_position.signals.values);
%     if size(gt_pos,1)==3, gt_pos=gt_pos'; end
%     GT_x = interp1(out.GT_position.time, gt_pos(:,1), imu_time,'linear','extrap');
%     GT_y = interp1(out.GT_position.time, gt_pos(:,2), imu_time,'linear','extrap');
% 
%     rmse = sqrt(mean((X_all(:,1)-GT_x).^2 + (X_all(:,2)-GT_y).^2));
%     results(d) = rmse;
%     fprintf('%s: %.4f m\n', datasets{d,2}, rmse);
% end
% 
% fprintf('\n=== Summary ===\n');
% fprintf('Task 1: D1=%.4f D2=%.4f D3=%.4f D4=%.4f  avg=%.4fm  [≤0.05m: %s]\n', ...
%     results(1), results(2), results(3), results(4), mean(results(1:4)), ...
%     ternary(mean(results(1:4)) <= 0.05, 'PASS', 'FAIL'));
% fprintf('Task 2: D1=%.4f D2=%.4f D3=%.4f D4=%.4f  avg=%.4fm  [≤0.30m: %s]\n', ...
%     results(5), results(6), results(7), results(8), mean(results(5:8)), ...
%     ternary(mean(results(5:8)) <= 0.30, 'PASS', 'FAIL'));
% 
% function s = ternary(c, a, b)
%     if c, s = a; else, s = b; end
% end

clear all; clc;

datasets = {
    'Training Data/task1_1.mat', 'T1_D1';
    'Training Data/task1_2.mat', 'T1_D2';
    'Training Data/task1_3.mat', 'T1_D3';
    'Training Data/task1_4.mat', 'T1_D4';
    'Training Data/task2_1.mat', 'T2_D1';
    'Training Data/task2_2.mat', 'T2_D2';
    'Training Data/task2_3.mat', 'T2_D3';
    'Training Data/task2_4.mat', 'T2_D4';
};

results = zeros(8, 1);
X_results = cell(8, 1);
GT_results = cell(8, 1);
time_results = cell(8, 1);

for d = 1:8
    clear myEKF;
    load(datasets{d, 1});

    gyro_data  = squeeze(out.Sensor_GYRO.signals.values)';
    accel_data = squeeze(out.Sensor_ACCEL.signals.values)';
    mag_data   = squeeze(out.Sensor_MAG.signals.values)';
    tof1_data  = squeeze(out.Sensor_ToF1.signals.values);
    tof2_data  = squeeze(out.Sensor_ToF2.signals.values);
    tof3_data  = squeeze(out.Sensor_ToF3.signals.values);

    N = size(gyro_data, 1);
    X_all = zeros(N, 8);

    for k = 1:N
        [xe, ~] = myEKF(accel_data(k,:)', gyro_data(k,:)', mag_data(k,:)', ...
                        tof1_data(k,:)', tof2_data(k,:)', tof3_data(k,:)', 0, 0);
        X_all(k,:) = xe';
    end

    imu_time = out.Sensor_GYRO.time;
    gt_pos   = squeeze(out.GT_position.signals.values);
    if size(gt_pos,1)==3, gt_pos=gt_pos'; end
    GT_x = interp1(out.GT_position.time, gt_pos(:,1), imu_time,'linear','extrap');
    GT_y = interp1(out.GT_position.time, gt_pos(:,2), imu_time,'linear','extrap');

    rmse = sqrt(mean((X_all(:,1)-GT_x).^2 + (X_all(:,2)-GT_y).^2));
    results(d) = rmse;
    X_results{d} = X_all;
    GT_results{d} = [GT_x, GT_y];
    time_results{d} = imu_time;

    fprintf('%s: %.4f m\n', datasets{d,2}, rmse);
end

fprintf('\n=== Summary ===\n');
fprintf('Task 1: D1=%.4f D2=%.4f D3=%.4f D4=%.4f  avg=%.4fm  [≤0.05m: %s]\n', ...
    results(1), results(2), results(3), results(4), mean(results(1:4)), ...
    ternary(mean(results(1:4)) <= 0.05, 'PASS', 'FAIL'));
fprintf('Task 2: D1=%.4f D2=%.4f D3=%.4f D4=%.4f  avg=%.4fm  [≤0.30m: %s]\n', ...
    results(5), results(6), results(7), results(8), mean(results(5:8)), ...
    ternary(mean(results(5:8)) <= 0.30, 'PASS', 'FAIL'));

%% Plot all 8 datasets
figure('Name', 'Task 1 Trajectories', 'Position', [100 100 1400 700]);
for d = 1:4
    subplot(2, 4, d);
    X_all = X_results{d};
    GT    = GT_results{d};
    t     = time_results{d};
    ox = GT(1,1); oy = GT(1,2);
    plot(GT(:,1)-ox, GT(:,2)-oy, 'k-', 'LineWidth', 2); hold on;
    plot(X_all(:,1)-ox, X_all(:,2)-oy, 'b--', 'LineWidth', 1.5);
    grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('%s  RMSE=%.3fm', datasets{d,2}, results(d)));
    legend('GT','EKF','Location','best');

    subplot(2, 4, d+4);
    pos_err = sqrt((X_all(:,1)-GT(:,1)).^2 + (X_all(:,2)-GT(:,2)).^2);
    plot(t, pos_err, 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Error (m)');
    title(sprintf('%s error over time', datasets{d,2}));
    grid on;
    yline(0.05, 'k--', 'Target');
end

figure('Name', 'Task 2 Trajectories', 'Position', [100 100 1400 700]);
for d = 5:8
    subplot(2, 4, d-4);
    X_all = X_results{d};
    GT    = GT_results{d};
    t     = time_results{d};
    ox = GT(1,1); oy = GT(1,2);
    plot(GT(:,1)-ox, GT(:,2)-oy, 'k-', 'LineWidth', 2); hold on;
    plot(X_all(:,1)-ox, X_all(:,2)-oy, 'b--', 'LineWidth', 1.5);
    grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('%s  RMSE=%.3fm', datasets{d,2}, results(d)));
    legend('GT','EKF','Location','best');

    subplot(2, 4, d);
    pos_err = sqrt((X_all(:,1)-GT(:,1)).^2 + (X_all(:,2)-GT(:,2)).^2);
    plot(t, pos_err, 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Error (m)');
    title(sprintf('%s error over time', datasets{d,2}));
    grid on;
    yline(0.30, 'k--', 'Target');
end

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end


% clear all;
% load(['Training Data/task2_4 1.mat']);
% imu_time = out.Sensor_GYRO.time;
% accel_data = squeeze(out.Sensor_ACCEL.signals.values)';
% gyro_data  = squeeze(out.Sensor_GYRO.signals.values)';
% 
% % Simulate ZUPT detection window
% accel_bias_fwd = -0.396;
% accel_bias_lat =  0.07485;
% 
% fprintf('=== ZUPT variance check t=5.5-7.0s ===\n');
% for t = 5.5:0.1:7.0
%     idx = find(imu_time>=t,1);
%     if idx < 40, continue; end
%     win = idx-39:idx;
%     ax_win = accel_data(win,3) - accel_bias_fwd;
%     ay_win = accel_data(win,2) - accel_bias_lat;
%     gy_win = gyro_data(win,1);
%     ax_win = max(-2.0, min(2.0, ax_win));
%     ay_win = max(-2.0, min(2.0, ay_win));
%     az_var = var(ax_win);
%     ay_var = var(ay_win);
%     gv     = var(gy_win);
%     fires  = az_var<0.005 && ay_var<0.005 && gv<0.0005;
%     fprintf('t=%.1fs: az_var=%.5f ay_var=%.5f gyro_var=%.6f ZUPT=%s\n',...
%         t, az_var, ay_var, gv, ternary(fires,'FIRE','no'));
% end
% 
% function s = ternary(c,a,b)
%     if c, s=a; else, s=b; end
% end