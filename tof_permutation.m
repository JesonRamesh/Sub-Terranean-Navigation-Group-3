% tof_diagnostics.m
% Runs the EKF and reports:
%   1. How many ToF updates were accepted vs rejected per sensor
%   2. Innovation statistics (mean, std) per sensor
%   3. Kalman gain magnitude over time
%   4. How much each ToF update actually moved the position estimate
% This tells us whether ToF is correcting position at all.

clear; clc; close all;
load('Training Data/task2_1 1.mat');

%% Run with diagnostics embedded
imu_time   = out.Sensor_GYRO.time;
gyro_data  = squeeze(out.Sensor_GYRO.signals.values)';
accel_data = squeeze(out.Sensor_ACCEL.signals.values)';
mag_time   = out.Sensor_MAG.time;
mag_data   = squeeze(out.Sensor_MAG.signals.values)';

tof_times = {out.Sensor_ToF1.time, out.Sensor_ToF2.time, out.Sensor_ToF3.time};
tof_raw   = {squeeze(out.Sensor_ToF1.signals.values), ...
             squeeze(out.Sensor_ToF2.signals.values), ...
             squeeze(out.Sensor_ToF3.signals.values)};
for i = 1:3
    if size(tof_raw{i}, 1) == 4, tof_raw{i} = tof_raw{i}'; end
end

gt_pos = squeeze(out.GT_position.signals.values);
if size(gt_pos, 1) == 3, gt_pos = gt_pos'; end

static_idx = imu_time < (imu_time(1) + 0.5);
gyro_bias  = mean(gyro_data(static_idx, :), 1);

mag_hard = [-3.705e-05, 4.415e-05];
mag_soft = [1.0958, 0.9196];

gt_rot = squeeze(out.GT_rotation.signals.values);
if size(gt_rot, 1) == 4, gt_rot = gt_rot'; end
qw = gt_rot(1,1); qx = gt_rot(1,2); qy = gt_rot(1,3); qz = gt_rot(1,4);
theta0 = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));

X = [gt_pos(1,1); gt_pos(1,2); theta0; 0; 0];
P = diag([0.01, 0.01, 0.01, 0.1, 0.1]);
Q = diag([1e-4, 1e-4, 1e-7, 0.5, 0.5]);
R_tof = 0.005;
R_mag = 0.5;
gate_threshold = 6.63;
gate_abs = 0.5;
vel_decay = 0.981;
bounds = struct('x_max', 1.2, 'x_min', -1.2, 'y_max', 1.65, 'y_min', -1.65);
alpha_tofs = [0, -pi/2, pi/2];

% Diagnostic accumulators
tof_accepted  = [0 0 0];
tof_rejected_gate = [0 0 0];
tof_rejected_status = [0 0 0];
tof_innovations = {[], [], []};
tof_gains_x = {[], [], []};   % how much x moved per update
tof_gains_y = {[], [], []};   % how much y moved per update
tof_expected = {[], [], []};  % expected vs measured distance
tof_measured  = {[], [], []};

num_steps = length(imu_time);
X_Est = zeros(num_steps, 5);
mag_idx = 1; tof_idxs = [1,1,1];
prev_time = imu_time(1);

for k = 1:num_steps
    dt = imu_time(k) - prev_time;
    if dt <= 0, dt = 0.001; end
    prev_time = imu_time(k);

    omega = 1.18*(gyro_data(k, 1) - gyro_bias(1));
    vx_new = vel_decay * X(4);
    vy_new = vel_decay * X(5);
    X(1) = X(1) + (X(4)*cos(X(3)) - X(5)*sin(X(3)))*dt;
    X(2) = X(2) + (X(4)*sin(X(3)) + X(5)*cos(X(3)))*dt;
    X(3) = wrapToPi(X(3) + omega*dt);
    X(4) = vx_new; X(5) = vy_new;

    F = eye(5);
    F(1,3) = (-X(4)*sin(X(3)) - X(5)*cos(X(3)))*dt;
    F(1,4) = cos(X(3))*dt; F(1,5) = -sin(X(3))*dt;
    F(2,3) = (X(4)*cos(X(3)) - X(5)*sin(X(3)))*dt;
    F(2,4) = sin(X(3))*dt; F(2,5) = cos(X(3))*dt;
    F(4,4) = vel_decay; F(5,5) = vel_decay;
    P = F * P * F' + Q;

    if mag_idx <= length(mag_time) && imu_time(k) >= mag_time(mag_idx)
        my = (mag_data(mag_idx,2) - mag_hard(1)) * mag_soft(1);
        mz = (mag_data(mag_idx,3) - mag_hard(2)) * mag_soft(2);
        z_mag = wrapToPi(atan2(mz, my));
        if mag_idx == 1, mag_offset = wrapToPi(theta0 - z_mag); end
        inn_mag = wrapToPi((z_mag + mag_offset) - X(3));
        H_mag = [0,0,1,0,0];
        S_mag = H_mag*P*H_mag' + R_mag;
        K_mag = P*H_mag'/S_mag;
        X = X + K_mag*inn_mag;
        P = (eye(5)-K_mag*H_mag)*P*(eye(5)-K_mag*H_mag)' + K_mag*R_mag*K_mag';
        mag_idx = mag_idx + 1;
    end

    for i = 1:3
        tidx = tof_idxs(i);
        if tidx <= length(tof_times{i}) && imu_time(k) >= tof_times{i}(tidx)
            data = tof_raw{i}(tidx, :);
            if data(4) ~= 0 || data(1) <= 0.05 || data(1) >= 4.0
                tof_rejected_status(i) = tof_rejected_status(i) + 1;
            else
                [h, H] = ray_cast_diag(X, bounds, alpha_tofs(i));
                inn = data(1) - h;
                S_tof = H*P*H' + R_tof;

                tof_expected{i}(end+1) = h;
                tof_measured{i}(end+1)  = data(1);

                if (inn^2 / S_tof) >= gate_threshold || abs(inn) >= gate_abs
                    tof_rejected_gate(i) = tof_rejected_gate(i) + 1;
                    tof_innovations{i}(end+1) = inn;  % log rejected too
                else
                    tof_accepted(i) = tof_accepted(i) + 1;
                    tof_innovations{i}(end+1) = inn;
                    x_before = X(1); y_before = X(2);
                    K_tof = P*H'/S_tof;
                    X = X + K_tof*inn;
                    X(3) = wrapToPi(X(3));
                    P = (eye(5)-K_tof*H)*P*(eye(5)-K_tof*H)' + K_tof*R_tof*K_tof';
                    tof_gains_x{i}(end+1) = X(1) - x_before;
                    tof_gains_y{i}(end+1) = X(2) - y_before;
                end
            end
            tof_idxs(i) = tidx + 1;
        end
    end
    X_Est(k,:) = X';
end

%% Report
fprintf('\n=== ToF Update Statistics ===\n');
fprintf('%-8s %-10s %-14s %-14s %-12s %-12s %-14s %-14s\n', ...
    'Sensor', 'Accepted', 'Rej(gate)', 'Rej(status)', ...
    'Inn mean', 'Inn std', 'dX mean', 'dY mean');
for i = 1:3
    if ~isempty(tof_gains_x{i})
        fprintf('ToF%-5d %-10d %-14d %-14d %-12.4f %-12.4f %-14.4f %-14.4f\n', ...
            i, tof_accepted(i), tof_rejected_gate(i), tof_rejected_status(i), ...
            mean(tof_innovations{i}), std(tof_innovations{i}), ...
            mean(tof_gains_x{i}), mean(tof_gains_y{i}));
    else
        fprintf('ToF%-5d %-10d %-14d %-14d %-12.4f %-12.4f  (no accepted updates)\n', ...
            i, tof_accepted(i), tof_rejected_gate(i), tof_rejected_status(i), ...
            mean(tof_innovations{i}), std(tof_innovations{i}));
    end
end

fprintf('\n=== Expected vs Measured Distance (mean over run) ===\n');
for i = 1:3
    if ~isempty(tof_expected{i})
        fprintf('ToF%d:  expected=%.3f m   measured=%.3f m   bias=%.3f m\n', ...
            i, mean(tof_expected{i}), mean(tof_measured{i}), ...
            mean(tof_measured{i}) - mean(tof_expected{i}));
    end
end

%% Plots
figure('Name','Innovation Histories');
sensor_names = {'ToF1 (forward)', 'ToF2 (left -90°)', 'ToF3 (right +90°)'};
for i = 1:3
    subplot(3,1,i);
    plot(tof_innovations{i}, 'b.'); hold on;
    yline(gate_abs, 'r--'); yline(-gate_abs, 'r--');
    title(sprintf('%s innovations (red = gate boundary)', sensor_names{i}));
    ylabel('inn (m)'); grid on;
end
xlabel('Update index');

figure('Name','Expected vs Measured Distance');
for i = 1:3
    subplot(3,1,i);
    plot(tof_expected{i}, 'k-'); hold on;
    plot(tof_measured{i}, 'b.');
    title(sprintf('%s: expected (black) vs measured (blue)', sensor_names{i}));
    ylabel('Distance (m)'); grid on;
end
xlabel('Update index');

%% Helpers
function [h, H] = ray_cast_diag(X, b, alpha)
    phi = wrapToPi(X(3) + alpha);
    cp = cos(phi); sp = sin(phi);
    d = [inf, inf, inf, inf];
    if cp >  1e-6, d(1) = (b.x_max - X(1))/cp; end
    if cp < -1e-6, d(2) = (b.x_min - X(1))/cp; end
    if sp >  1e-6, d(3) = (b.y_max - X(2))/sp; end
    if sp < -1e-6, d(4) = (b.y_min - X(2))/sp; end
    [h, idx] = min(d);
    H = zeros(1,5);
    if idx <= 2, H(1) = -1/cp; H(3) = h*(sp/cp);
    else,        H(2) = -1/sp; H(3) = -h*(cp/sp); end
end

function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end