clear all; clc;

datasets = {
    'Training Data/task1_1 1.mat', 'T1_D1';
    'Training Data/task1_2 1.mat', 'T1_D2';
    'Training Data/task1_3.mat', 'T1_D3';
    'Training Data/task2_1 1.mat', 'T2_D1';
    'Training Data/task2_2 1.mat', 'T2_D2';
    'Training Data/task2_3 1.mat', 'T2_D3';
};

results = zeros(6, 1);

for d = 1:6
    clear myEKF;  % Reset persistent variables for each dataset
    load(datasets{d, 1});

    gyro_data  = squeeze(out.Sensor_GYRO.signals.values)';
    accel_data = squeeze(out.Sensor_ACCEL.signals.values)';
    mag_data   = squeeze(out.Sensor_MAG.signals.values)';
    tof1_data  = squeeze(out.Sensor_ToF1.signals.values);
    tof2_data  = squeeze(out.Sensor_ToF2.signals.values);
    tof3_data  = squeeze(out.Sensor_ToF3.signals.values);

    N = size(gyro_data, 1);
    X_all = zeros(N, 5);

    for k = 1:N
        [xe, ~] = myEKF(accel_data(k,:)', gyro_data(k,:)', mag_data(k,:)', ...
                        tof1_data(k,:)', tof2_data(k,:)', tof3_data(k,:)', ...
                        0, 0);
        X_all(k,:) = xe;
    end

    imu_time = out.Sensor_GYRO.time;
    gt_pos   = squeeze(out.GT_position.signals.values);
    if size(gt_pos,1)==3, gt_pos=gt_pos'; end
    GT_x = interp1(out.GT_position.time, gt_pos(:,1), imu_time, 'linear', 'extrap');
    GT_y = interp1(out.GT_position.time, gt_pos(:,2), imu_time, 'linear', 'extrap');

    rmse = sqrt(mean((X_all(:,1)-GT_x).^2 + (X_all(:,2)-GT_y).^2));
    results(d) = rmse;
    fprintf('%s: %.4f m\n', datasets{d,2}, rmse);
end

fprintf('\n=== Summary ===\n');
fprintf('Task 1: D1=%.4fm  D2=%.4fm  D3=%.4fm  avg=%.4fm\n', ...
        results(1), results(2), results(3), mean(results(1:3)));
fprintf('Task 2: D1=%.4fm  D2=%.4fm  D3=%.4fm  avg=%.4fm\n', ...
        results(4), results(5), results(6), mean(results(4:6)));
fprintf('T1 target ≤0.05m: %s\n', ...
        ternary(mean(results(1:3))<=0.05, 'PASS', 'FAIL'));
fprintf('T2 target ≤0.30m: %s\n', ...
        ternary(mean(results(4:6))<=0.30, 'PASS', 'FAIL'));

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end