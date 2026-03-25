clear all; clc;

datasets = {
    'Training Data/task1_1 1.mat', 'T1_D1';
    'Training Data/task1_2 1.mat', 'T1_D2';
    'Training Data/task1_3.mat',   'T1_D3';
    'Training Data/task1_4.mat',   'T1_D4';
    'Training Data/task2_1 1.mat', 'T2_D1';
    'Training Data/task2_2 1.mat', 'T2_D2';
    'Training Data/task2_3 1.mat', 'T2_D3';
    'Training Data/task2_4.mat',   'T2_D4';
};

results      = zeros(8, 1);   % position RMSE
yaw_results  = zeros(8, 1);   % yaw RMSE (degrees)
X_results    = cell(8, 1);
GT_results   = cell(8, 1);
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

    %% Ground truth position (interpolated onto IMU time grid)
    gt_pos = squeeze(out.GT_position.signals.values);
    if size(gt_pos,1)==3, gt_pos=gt_pos'; end
    GT_x = interp1(out.GT_position.time, gt_pos(:,1), imu_time, 'linear', 'extrap');
    GT_y = interp1(out.GT_position.time, gt_pos(:,2), imu_time, 'linear', 'extrap');

    %% Ground truth yaw from quaternions (interpolated onto IMU time grid)
    % GT_rotation stores quaternions [w, x, y, z] at 200 Hz
    gt_rot = squeeze(out.GT_rotation.signals.values);
    if size(gt_rot,1)==4, gt_rot=gt_rot'; end   % ensure N x 4

    % Convert each quaternion to yaw (rotation about world Z axis)
    % yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    qw = gt_rot(:,1); qx = gt_rot(:,2);
    qy = gt_rot(:,3); qz = gt_rot(:,4);
    gt_yaw_raw = atan2(2*(qw.*qz + qx.*qy), 1 - 2*(qy.^2 + qz.^2)) + pi;
    gt_yaw_raw = atan2(sin(gt_yaw_raw), cos(gt_yaw_raw)); % rewrap to [-pi, pi]

    % Interpolate GT yaw onto IMU time grid using circular (sin/cos) interpolation
    % to avoid discontinuities at the +/-pi wrap boundary
    GT_yaw_sin = interp1(out.GT_rotation.time, sin(gt_yaw_raw), imu_time, 'linear', 'extrap');
    GT_yaw_cos = interp1(out.GT_rotation.time, cos(gt_yaw_raw), imu_time, 'linear', 'extrap');
    GT_yaw = atan2(GT_yaw_sin, GT_yaw_cos);   % reconstruct wrapped angle

    %% Position RMSE
    pos_rmse = sqrt(mean((X_all(:,1)-GT_x).^2 + (X_all(:,2)-GT_y).^2));
    results(d) = pos_rmse;

    %% Yaw RMSE
    % wrapToPi difference handles the +/-pi discontinuity
    yaw_err_rad = wrapToPi(X_all(:,3) - GT_yaw);
    yaw_rmse_deg = sqrt(mean(yaw_err_rad.^2)) * (180/pi);
    yaw_results(d) = yaw_rmse_deg;

    X_results{d}    = X_all;
    GT_results{d}   = [GT_x, GT_y, GT_yaw];
    time_results{d} = imu_time;

    fprintf('%s: pos RMSE=%.4f m   yaw RMSE=%.2f deg\n', datasets{d,2}, pos_rmse, yaw_rmse_deg);
end

fprintf('\n=== Summary ===\n');
fprintf('Task 1: D1=%.4f D2=%.4f D3=%.4f D4=%.4f  avg=%.4fm  [<=0.05m: %s]\n', ...
    results(1), results(2), results(3), results(4), mean(results(1:4)), ...
    ternary(mean(results(1:4)) <= 0.05, 'PASS', 'FAIL'));
fprintf('Task 2: D1=%.4f D2=%.4f D3=%.4f D4=%.4f  avg=%.4fm  [<=0.30m: %s]\n', ...
    results(5), results(6), results(7), results(8), mean(results(5:8)), ...
    ternary(mean(results(5:8)) <= 0.30, 'PASS', 'FAIL'));

fprintf('\n--- Yaw RMSE (degrees) ---\n');
fprintf('Task 1: D1=%.2f D2=%.2f D3=%.2f D4=%.2f  avg=%.2f deg\n', ...
    yaw_results(1), yaw_results(2), yaw_results(3), yaw_results(4), mean(yaw_results(1:4)));
fprintf('Task 2: D1=%.2f D2=%.2f D3=%.2f D4=%.2f  avg=%.2f deg\n', ...
    yaw_results(5), yaw_results(6), yaw_results(7), yaw_results(8), mean(yaw_results(5:8)));

%% Plot all 8 datasets — trajectory + position error + yaw error
figure('Name', 'Task 1', 'Position', [50 50 1600 800]);
for d = 1:4
    X_all = X_results{d};
    GT    = GT_results{d};
    t     = time_results{d};
    ox = GT(1,1); oy = GT(1,2);

    subplot(3, 4, d);
    plot(GT(:,1)-ox, GT(:,2)-oy, 'k-', 'LineWidth', 2); hold on;
    plot(X_all(:,1)-ox, X_all(:,2)-oy, 'b--', 'LineWidth', 1.5);
    grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('%s  pos=%.3fm', datasets{d,2}, results(d)));
    legend('GT','EKF','Location','best');

    subplot(3, 4, d+4);
    pos_err = sqrt((X_all(:,1)-GT(:,1)).^2 + (X_all(:,2)-GT(:,2)).^2);
    plot(t, pos_err, 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Pos error (m)');
    title(sprintf('%s pos error', datasets{d,2}));
    grid on;
    yline(0.05, 'k--', 'Target');

    subplot(3, 4, d+8);
    yaw_err_deg = wrapToPi(X_all(:,3) - GT(:,3)) * (180/pi);
    plot(t, yaw_err_deg, 'm-', 'LineWidth', 1.2);
    xlabel('Time (s)'); ylabel('Yaw error (deg)');
    title(sprintf('%s yaw RMSE=%.1fdeg', datasets{d,2}, yaw_results(d)));
    grid on;
    yline(0, 'k--');
end

figure('Name', 'Task 2', 'Position', [50 50 1600 800]);
for d = 5:8
    X_all = X_results{d};
    GT    = GT_results{d};
    t     = time_results{d};
    ox = GT(1,1); oy = GT(1,2);

    subplot(3, 4, d-4);
    plot(GT(:,1)-ox, GT(:,2)-oy, 'k-', 'LineWidth', 2); hold on;
    plot(X_all(:,1)-ox, X_all(:,2)-oy, 'b--', 'LineWidth', 1.5);
    grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('%s  pos=%.3fm', datasets{d,2}, results(d)));
    legend('GT','EKF','Location','best');

    subplot(3, 4, d);
    pos_err = sqrt((X_all(:,1)-GT(:,1)).^2 + (X_all(:,2)-GT(:,2)).^2);
    plot(t, pos_err, 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Pos error (m)');
    title(sprintf('%s pos error', datasets{d,2}));
    grid on;
    yline(0.30, 'k--', 'Target');

    subplot(3, 4, d+4);
    yaw_err_deg = wrapToPi(X_all(:,3) - GT(:,3)) * (180/pi);
    plot(t, yaw_err_deg, 'm-', 'LineWidth', 1.2);
    xlabel('Time (s)'); ylabel('Yaw error (deg)');
    title(sprintf('%s yaw RMSE=%.1fdeg', datasets{d,2}, yaw_results(d)));
    grid on;
    yline(0, 'k--');
end

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end
