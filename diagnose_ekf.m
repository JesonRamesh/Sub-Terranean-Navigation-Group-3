%% diagnose_ekf.m — Run this on a task dataset to see what's going wrong
%
%  Usage: Load a task .mat file, then run this script.
%         e.g.:  load('task1_1_extracted.mat');  diagnose_ekf;
%
%  This script does NOT run the EKF. It compares raw sensor data against
%  ground truth to identify axis mapping issues, yaw offsets, and ToF
%  mounting problems.

% If data is nested in 'out' variable
if exist('out','var')
    data = out;
else
    error('Load a task .mat file first (should create "out" variable)');
end

%% Extract everything
sensor_time = data.Sensor_Time.time(:);
N = length(sensor_time);

gyro  = squeeze(data.Sensor_GYRO.signals.values);
accel = squeeze(data.Sensor_ACCEL.signals.values);
mag   = squeeze(data.Sensor_MAG.signals.values);
if size(gyro,1)==3  && size(gyro,2)==N,  gyro=gyro';   end
if size(accel,1)==3 && size(accel,2)==N, accel=accel';  end
if size(mag,1)==3   && size(mag,2)==N,   mag=mag';      end

tof1 = squeeze(data.Sensor_ToF1.signals.values);
tof2 = squeeze(data.Sensor_ToF2.signals.values);
tof3 = squeeze(data.Sensor_ToF3.signals.values);
if size(tof1,1)==4 && size(tof1,2)==N, tof1=tof1'; end
if size(tof2,1)==4 && size(tof2,2)==N, tof2=tof2'; end
if size(tof3,1)==4 && size(tof3,2)==N, tof3=tof3'; end

gt_pos = squeeze(data.GT_position.signals.values);
gt_rot = squeeze(data.GT_rotation.signals.values);
if size(gt_pos,1)==3 && size(gt_pos,2)==N, gt_pos=gt_pos'; end
if size(gt_rot,1)==4 && size(gt_rot,2)==N, gt_rot=gt_rot'; end

gt_yaw = atan2(2*(gt_rot(:,1).*gt_rot(:,4) + gt_rot(:,2).*gt_rot(:,3)), ...
               1 - 2*(gt_rot(:,3).^2 + gt_rot(:,4).^2));

%% Calibration values (from calibrate.m output)
GYRO_BIAS = [-0.49032; -0.02242; 0.01574];

%% ===== PLOT 1: Ground Truth Trajectory =====
figure('Name','1. Ground Truth Trajectory');
plot(gt_pos(:,1), gt_pos(:,2), 'k-', 'LineWidth', 2);
hold on;
plot(gt_pos(1,1), gt_pos(1,2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(gt_pos(end,1), gt_pos(end,2), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
% Draw arena
rectangle('Position', [-1.22 -1.22 2.44 2.44], 'EdgeColor', 'r', 'LineStyle', '--');
xlabel('X [m]'); ylabel('Y [m]');
title('Ground Truth Position (green=start, red=end)');
axis equal; grid on;
legend('GT path','Start','End','Arena');

%% ===== PLOT 2: GT Yaw over time =====
figure('Name','2. Ground Truth Yaw');
plot(sensor_time, rad2deg(gt_yaw), 'k-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Yaw [deg]');
title('Ground Truth Yaw Angle');
grid on;

fprintf('\n=== GT YAW ===\n');
fprintf('Initial yaw: %.2f deg\n', rad2deg(gt_yaw(6)));
fprintf('Mean yaw:    %.2f deg\n', rad2deg(mean(gt_yaw(6:end))));
fprintf('Yaw range:   %.2f to %.2f deg\n', rad2deg(min(gt_yaw(6:end))), rad2deg(max(gt_yaw(6:end))));

%% ===== PLOT 3: Gyro integrated yaw vs GT yaw =====
figure('Name','3. Gyro Yaw vs GT Yaw');

% Integrate gyro axis 1 (yaw axis) with bias correction
dt_vec = diff(sensor_time);
gyro_yaw_integrated = zeros(N,1);
gyro_yaw_integrated(6) = gt_yaw(6);  % init from GT
for i = 7:N
    ddt = dt_vec(i-1);
    if ddt > 0 && ddt < 0.5
        gyro_yaw_integrated(i) = gyro_yaw_integrated(i-1) + (gyro(i,1) - GYRO_BIAS(1)) * ddt;
    else
        gyro_yaw_integrated(i) = gyro_yaw_integrated(i-1);
    end
end

plot(sensor_time, rad2deg(gt_yaw), 'k-', 'LineWidth', 1.5); hold on;
plot(sensor_time, rad2deg(gyro_yaw_integrated), 'b--', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Yaw [deg]');
title('Gyro-integrated Yaw vs GT Yaw');
legend('GT yaw', 'Gyro integrated (axis 1, bias-corrected)');
grid on;

fprintf('\n=== GYRO YAW DRIFT ===\n');
yaw_error = rad2deg(gyro_yaw_integrated(end) - gt_yaw(end));
fprintf('Final yaw error (gyro - GT): %.2f deg\n', yaw_error);

%% ===== PLOT 4: ToF distances over time =====
figure('Name','4. ToF Distances');
subplot(3,1,1);
plot(sensor_time, tof1(:,1), 'b.', 'MarkerSize', 2);
ylabel('ToF1 [m]'); title('ToF1 distance'); grid on;
subplot(3,1,2);
plot(sensor_time, tof2(:,1), 'r.', 'MarkerSize', 2);
ylabel('ToF2 [m]'); title('ToF2 distance'); grid on;
subplot(3,1,3);
plot(sensor_time, tof3(:,1), 'g.', 'MarkerSize', 2);
ylabel('ToF3 [m]'); title('ToF3 distance'); grid on;
xlabel('Time [s]');

%% ===== PLOT 5: Expected ToF vs measured — the KEY diagnostic =====
%  Using GT position and yaw, compute what each ToF SHOULD read
%  for different mounting angle hypotheses

WALLS = struct('xp', 1.22, 'xn', -1.22, 'yp', 1.22, 'yn', -1.22);

% Test multiple yaw offset values
yaw_offsets_deg = [0, 1.72, 5.7, -5.7, 90, -90, 180];
% Test multiple ToF mounting hypotheses
%  Hypothesis A: ToF1=fwd(0), ToF2=left(+90), ToF3=right(-90)
%  Hypothesis B: ToF1=left(+90), ToF2=back(180), ToF3=right(-90)  [original spec]
%  Hypothesis C: ToF1=fwd(0), ToF2=right(-90), ToF3=left(+90)
tof_hypotheses = {
    [0, pi/2, -pi/2],    'A: fwd, left, right';
    [pi/2, pi, -pi/2],   'B: left, back, right';
    [0, -pi/2, pi/2],    'C: fwd, right, left';
    [pi, pi/2, -pi/2],   'D: back, left, right';
    [0, pi, -pi/2],      'E: fwd, back, right';
    [0, pi/2, pi],        'F: fwd, left, back';
};

fprintf('\n=== TOF MOUNTING HYPOTHESIS TEST ===\n');
fprintf('Testing which ToF angle assignment best matches measured distances...\n');
fprintf('Using GT position/yaw + each hypothesis to predict distances.\n\n');

% Subsample for speed (ToF is ~10Hz anyway)
idx = 6:10:N;

best_rmse = Inf;
best_hyp = '';
best_offset = 0;

for h = 1:size(tof_hypotheses,1)
    angles = tof_hypotheses{h,1};
    hyp_name = tof_hypotheses{h,2};

    for oi = 1:length(yaw_offsets_deg)
        yoff = deg2rad(yaw_offsets_deg(oi));

        errors_all = [];
        for s = 1:3
            switch s
                case 1, tof_meas = tof1(idx,1);
                case 2, tof_meas = tof2(idx,1);
                case 3, tof_meas = tof3(idx,1);
            end

            tof_pred = zeros(size(tof_meas));
            for j = 1:length(idx)
                k = idx(j);
                ray = gt_yaw(k) + yoff + angles(s);
                [d, ~] = predictToFDist(gt_pos(k,1), gt_pos(k,2), ray, WALLS);
                tof_pred(j) = d;
            end

            valid = ~isnan(tof_pred) & tof_meas > 0.02 & tof_meas < 2.5;
            if any(valid)
                errors_all = [errors_all; tof_meas(valid) - tof_pred(valid)];
            end
        end

        if ~isempty(errors_all)
            rmse = sqrt(mean(errors_all.^2));
            if rmse < best_rmse
                best_rmse = rmse;
                best_hyp = hyp_name;
                best_offset = yaw_offsets_deg(oi);
            end
            if rmse < 0.3  % Only print promising ones
                fprintf('  %s | offset=%+6.1f deg | RMSE=%.4f m\n', hyp_name, yaw_offsets_deg(oi), rmse);
            end
        end
    end
end

fprintf('\n  *** BEST: %s | offset=%+.1f deg | RMSE=%.4f m ***\n', best_hyp, best_offset, best_rmse);

% Now plot the best hypothesis
best_angles_idx = find(strcmp(tof_hypotheses(:,2), best_hyp));
best_angles = tof_hypotheses{best_angles_idx, 1};
best_yoff = deg2rad(best_offset);

figure('Name','5. ToF Predicted vs Measured (Best Hypothesis)');
tof_labels = {'ToF1','ToF2','ToF3'};
tof_data = {tof1, tof2, tof3};
for s = 1:3
    subplot(3,1,s);
    tof_m = tof_data{s}(idx, 1);
    tof_p = zeros(size(tof_m));
    for j = 1:length(idx)
        k = idx(j);
        ray = gt_yaw(k) + best_yoff + best_angles(s);
        [d, ~] = predictToFDist(gt_pos(k,1), gt_pos(k,2), ray, WALLS);
        tof_p(j) = d;
    end
    plot(sensor_time(idx), tof_m, 'b.', 'MarkerSize', 4); hold on;
    plot(sensor_time(idx), tof_p, 'r.', 'MarkerSize', 4);
    ylabel([tof_labels{s} ' [m]']);
    title(sprintf('%s — Measured (blue) vs Predicted (red)', tof_labels{s}));
    grid on;
    legend('Measured','Predicted (GT-based)');
end
xlabel('Time [s]');
sgtitle(sprintf('Best: %s | YawOffset=%+.1f°', best_hyp, best_offset));

%% ===== PLOT 6: Accel during task =====
figure('Name','6. Accelerometer');
subplot(3,1,1);
plot(sensor_time, accel(:,1)); ylabel('Axis 1 [m/s²]');
title('Accel Axis 1 (gravity)'); grid on;
subplot(3,1,2);
plot(sensor_time, accel(:,2)); ylabel('Axis 2 [m/s²]');
title('Accel Axis 2'); grid on;
subplot(3,1,3);
plot(sensor_time, accel(:,3)); ylabel('Axis 3 [m/s²]');
title('Accel Axis 3'); grid on;
xlabel('Time [s]');

%% ===== PLOT 7: Mag heading vs GT yaw =====
figure('Name','7. Mag Heading vs GT Yaw');
MAG_HI = [4.45e-06; -4.84e-05; 8.90e-05];
mag_corrected_2 = mag(:,2) - MAG_HI(2);
mag_corrected_3 = mag(:,3) - MAG_HI(3);
mag_heading = atan2(mag_corrected_3, mag_corrected_2);

plot(sensor_time, rad2deg(gt_yaw), 'k-', 'LineWidth', 1.5); hold on;
plot(sensor_time, rad2deg(mag_heading), 'b.', 'MarkerSize', 2);
xlabel('Time [s]'); ylabel('Angle [deg]');
title('Magnetometer heading vs GT yaw');
legend('GT yaw', 'Mag heading (atan2(axis3, axis2))');
grid on;

% Also try swapped axes
mag_heading_swap = atan2(mag_corrected_2, mag_corrected_3);
figure('Name','7b. Mag Heading (swapped) vs GT Yaw');
plot(sensor_time, rad2deg(gt_yaw), 'k-', 'LineWidth', 1.5); hold on;
plot(sensor_time, rad2deg(mag_heading_swap), 'b.', 'MarkerSize', 2);
xlabel('Time [s]'); ylabel('Angle [deg]');
title('Magnetometer heading (SWAPPED: atan2(axis2, axis3)) vs GT yaw');
legend('GT yaw', 'Mag heading swapped');
grid on;

fprintf('\n=== DIAGNOSTICS COMPLETE ===\n');
fprintf('Look at the plots to identify:\n');
fprintf('  Plot 3: Does gyro yaw track GT? If not, bias is wrong\n');
fprintf('  Plot 5: Do predicted ToF match measured? If not, mounting angles or yaw offset are wrong\n');
fprintf('  Plot 6: Which accel axis responds during forward motion?\n');
fprintf('  Plot 7: Which mag heading formula tracks GT yaw?\n');

%% ===== Helper =====
function [d, wall_id] = predictToFDist(px, py, ray_angle, W)
    cos_a = cos(ray_angle);
    sin_a = sin(ray_angle);
    d = Inf; wall_id = 0;

    if cos_a > 1e-6
        t = (W.xp - px) / cos_a;
        hy = py + t * sin_a;
        if t > 0 && hy >= W.yn && hy <= W.yp && t < d, d = t; wall_id = 1; end
    end
    if cos_a < -1e-6
        t = (W.xn - px) / cos_a;
        hy = py + t * sin_a;
        if t > 0 && hy >= W.yn && hy <= W.yp && t < d, d = t; wall_id = 2; end
    end
    if sin_a > 1e-6
        t = (W.yp - py) / sin_a;
        hx = px + t * cos_a;
        if t > 0 && hx >= W.xn && hx <= W.xp && t < d, d = t; wall_id = 3; end
    end
    if sin_a < -1e-6
        t = (W.yn - py) / sin_a;
        hx = px + t * cos_a;
        if t > 0 && hx >= W.xn && hx <= W.xp && t < d, d = t; wall_id = 4; end
    end

    if isinf(d), d = NaN; wall_id = 0; end
end