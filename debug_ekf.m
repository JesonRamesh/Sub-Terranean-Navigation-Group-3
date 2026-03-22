% debug_ekf.m  —  EKF root-cause diagnostics
% Run from the Coursework directory.  Prints numbers and produces 4 figures.
clear; clc; close all;

load('Training Data/task1_1 1.mat');

% -------------------------------------------------------------------------
% 1.  Re-run the filter and pull out the already-stored outputs
% -------------------------------------------------------------------------
[X_Est, P_Est] = myEKF(out);
gt_pos = squeeze(out.GT_position.signals.values);
if size(gt_pos, 1) == 3, gt_pos = gt_pos'; end
GT = gt_pos;

imu_time = out.Sensor_GYRO.time;
n        = min(length(imu_time), size(X_Est,1));
t        = imu_time(1:n) - imu_time(1);          % seconds from 0

origin   = GT(1,1:2);
GT_rel   = GT(:,1:2) - origin;
X_rel    = X_Est(1:n,1:2) - origin;

% -------------------------------------------------------------------------
% 2.  Extract ToF2 data for manual analysis
% -------------------------------------------------------------------------
tof2_time = out.Sensor_ToF2.time;
tof2_data = squeeze(out.Sensor_ToF2.signals.values);
if size(tof2_data,1)==4, tof2_data = tof2_data'; end

arena_y_max =  1.2;
arena_y_min = -2.16;

% Compute GT-based expected ToF2 distance (theta ~ 0, sensor faces +Y)
gt_time_raw = out.GT_time.signals.values;
gt_y_abs    = GT(:,2);   % absolute y from GT

% For each GT sample, expected ToF2 = y_max - y  (when theta ≈ 0)
tof2_expected_from_GT = arena_y_max - gt_y_abs;

% Unique ToF2 raw readings (non-repeated = genuine sensor values)
tof2_raw_all   = tof2_data(:,1);
tof2_valid_all = tof2_data(:,4) == 0;   % valid flag

% -------------------------------------------------------------------------
% 3.  Print key scalar diagnostics
% -------------------------------------------------------------------------
vy_est  = X_Est(1:n, 5);
P55     = squeeze(P_Est(5,5,1:n));

fprintf('=== EKF Diagnostics ===\n\n');
fprintf('-- Velocity estimates (body vy, should be ~0.1 m/s during motion) --\n');
fprintf('  Initial vy_est     : %+.4f m/s\n', vy_est(1));
fprintf('  Median  |vy_est|   : %.4f m/s\n', median(abs(vy_est)));
fprintf('  Max     |vy_est|   : %.4f m/s  (sane limit ≈ 0.5 m/s)\n', max(abs(vy_est)));
fprintf('  Std     vy_est     : %.4f m/s\n', std(vy_est));

fprintf('\n-- Velocity covariance P(5,5) --\n');
fprintf('  Initial P(5,5)     : %.4f\n', P55(1));
fprintf('  Max     P(5,5)     : %.2f   (if >> 4, Q_vy is too large)\n', max(P55));
fprintf('  Median  P(5,5)     : %.4f\n', median(P55));

fprintf('\n-- Y-position tracking --\n');
n_min = min(size(GT_rel,1), n);
fprintf('  GT final y (rel)   : %.4f m\n', GT_rel(n_min,2));
fprintf('  EKF final y (rel)  : %.4f m\n', X_rel(n_min,2));
fprintf('  Ratio (EKF/GT)     : %.3f  (1.0 = perfect, 0.5 = 50%% tracking)\n', ...
        X_rel(n_min,2)/GT_rel(n_min,2));

fprintf('\n-- ToF2 sensor vs arena model --\n');
fprintf('  Robot start y (abs): %.4f m\n', gt_y_abs(1));
fprintf('  Robot end   y (abs): %.4f m\n', gt_y_abs(end));
fprintf('  Expected ToF2 start: %.4f m (y_max - y_start)\n', arena_y_max - gt_y_abs(1));
fprintf('  Expected ToF2 end  : %.4f m (y_max - y_end)\n',   arena_y_max - gt_y_abs(end));
fprintf('  Actual   ToF2 raw start: %.4f m\n', tof2_raw_all(1));
fprintf('  Actual   ToF2 raw end  : %.4f m\n', tof2_raw_all(end));

% ToF2 change vs GT y-change
gt_y_change  = gt_y_abs(end) - gt_y_abs(1);
tof2_change  = tof2_raw_all(end) - tof2_raw_all(1);
fprintf('  GT y change        : %.4f m\n', gt_y_change);
fprintf('  ToF2 raw change    : %.4f m (should be ~ -GT_y_change)\n', tof2_change);

fprintf('\n-- Mahalanobis gating check for ToF2 --\n');
% Recompute a quick innovation trace using GT positions
valid_mask = tof2_valid_all;
raw_valid  = tof2_raw_all(valid_mask);
n_tof2     = sum(valid_mask);
fprintf('  Total valid ToF2 samples: %d\n', n_tof2);
% Check unique values (ZOH check)
[~,ui] = unique(tof2_raw_all,'stable');
fprintf('  Unique raw values      : %d out of %d total\n', length(ui), length(tof2_raw_all));
fprintf('  ZOH repeat factor      : %.1fx\n', length(tof2_raw_all)/length(ui));

% -------------------------------------------------------------------------
% 4.  Figure 1 — Y position vs time
% -------------------------------------------------------------------------
figure(1);
plot(t(1:n_min), GT_rel(1:n_min,2), 'k-', 'LineWidth',2); hold on;
plot(t(1:n_min), X_rel(1:n_min,2),  'b--','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Y (m, rel to start)');
title('Y Position vs Time'); legend('GT','EKF'); grid on;

% -------------------------------------------------------------------------
% 5.  Figure 2 — vy estimate and P(5,5) over time
% -------------------------------------------------------------------------
figure(2);
subplot(2,1,1);
plot(t, vy_est, 'r'); hold on;
yline(0,'k--');
xlabel('Time (s)'); ylabel('vy_est (m/s)');
title('Body-frame lateral velocity estimate (vy)'); grid on;
subplot(2,1,2);
plot(t, P55, 'b');
xlabel('Time (s)'); ylabel('P(5,5)');
title('vy covariance — if this blows up, Q\_vy is too large'); grid on;

% -------------------------------------------------------------------------
% 6.  Figure 3 — ToF2 raw vs model prediction (using GT)
% -------------------------------------------------------------------------
% Down-sample GT to match imu_time for plotting
gt_ts = squeeze(out.GT_time.signals.values);
gt_ts = gt_ts(:);          % force column vector
gt_ts = gt_ts - gt_ts(1);
tof2_exp_interp = interp1(gt_ts, tof2_expected_from_GT, t, 'linear', 'extrap');

% Get ToF2 raw at same times (nearest-neighbour into tof2_time)
tof2_raw_interp = interp1(tof2_time, tof2_raw_all, t, 'nearest', 'extrap');

figure(3);
plot(t, tof2_raw_interp,   'g-',  'LineWidth',1.5,'DisplayName','ToF2 raw'); hold on;
plot(t, tof2_exp_interp,   'k--', 'LineWidth',1.5,'DisplayName','Expected (y\_max - GT\_y)');
plot(t, arena_y_max - X_rel(1:n,2) - origin(2), 'b:', 'LineWidth',1.5, ...
     'DisplayName','Predicted (y\_max - EKF\_y)');
xlabel('Time (s)'); ylabel('Distance to y\_max wall (m)');
title('ToF2 measurement vs model'); legend; grid on;

% -------------------------------------------------------------------------
% 7.  Figure 4 — Innovation (raw - predicted from EKF y)
% -------------------------------------------------------------------------
ekf_y_abs = X_Est(1:n,2);     % absolute y estimate
h_x_tof2  = arena_y_max - ekf_y_abs;
innov_tof2 = tof2_raw_interp - h_x_tof2;

figure(4);
subplot(2,1,1);
plot(t, innov_tof2, 'm'); hold on; yline(0,'k--');
xlabel('Time (s)'); ylabel('Innovation (m)');
title('ToF2 innovation: raw\_dist - (y\_max - y\_est)'); grid on;
subplot(2,1,2);
R_tof = 0.01;
P22   = squeeze(P_Est(2,2,1:n));
S_approx = P22 + R_tof;
maha  = innov_tof2.^2 ./ S_approx;
semilogy(t, maha, 'b'); hold on;
yline(100,'r--','LineWidth',2);
xlabel('Time (s)'); ylabel('\gamma = innov^2/S');
title('Mahalanobis score — updates above red line are GATED OUT'); grid on;

fprintf('\n--- Done.  Check figures for visual diagnosis. ---\n');
