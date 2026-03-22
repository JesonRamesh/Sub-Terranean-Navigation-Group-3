% debug_task2.m
% Diagnoses WHY Helitha's code (gamma=3.84, accel input) fails on Task 2.
% Run AFTER loading task2 data:  load('Training Data/task2_1 1.mat')
% Then: debug_task2

clear; clc; close all;
load('Training Data/task2_1 1.mat');

[X_Est, P_Est] = myEKF(out);
gt_pos = squeeze(out.GT_position.signals.values);
if size(gt_pos, 1) == 3, gt_pos = gt_pos'; end
GT = gt_pos;

imu_time = out.Sensor_GYRO.time;
t        = imu_time - imu_time(1);
N        = length(t);

origin   = GT(1,1:2);
GT_rel   = GT(:,1:2) - origin;
X_rel    = X_Est(:,1:2) - origin;
n        = min(size(GT_rel,1), N);

gyro_raw  = squeeze(out.Sensor_GYRO.signals.values)';
accel_raw = squeeze(out.Sensor_ACCEL.signals.values)';
dt_imu    = median(diff(t));

% Reconstruct omega the same way the EKF does
gyro_bias_x = 0.00186;
omega_est   = -(gyro_raw(:,1) - gyro_bias_x);   % rad/s, EKF convention

% -------------------------------------------------------------------------
% 1.  RMSE
% -------------------------------------------------------------------------
err_x    = X_rel(1:n,1) - GT_rel(1:n,1);
err_y    = X_rel(1:n,2) - GT_rel(1:n,2);
rmse_x   = sqrt(mean(err_x.^2));
rmse_y   = sqrt(mean(err_y.^2));
total    = sqrt(rmse_x^2 + rmse_y^2);

fprintf('=== Task 2 Diagnostics ===\n\n');
fprintf('RMSE X: %.4f m\n', rmse_x);
fprintf('RMSE Y: %.4f m\n', rmse_y);
fprintf('Total:  %.4f m\n\n', total);

% -------------------------------------------------------------------------
% 2.  GT trajectory shape — understand what Task 2 looks like
% -------------------------------------------------------------------------
fprintf('--- GT trajectory shape ---\n');
fprintf('Total run time        : %.1f s\n', t(n));
fprintf('Total GT path length  : %.2f m\n', sum(sqrt(diff(GT_rel(1:n,1)).^2 + diff(GT_rel(1:n,2)).^2)));
fprintf('Start: (%.3f, %.3f)  End: (%.3f, %.3f)\n', GT_rel(1,1), GT_rel(1,2), GT_rel(n,1), GT_rel(n,2));

% Find turning phases from GT heading
gt_rot = squeeze(out.GT_rotation.signals.values);
if size(gt_rot,1)==4, gt_rot=gt_rot'; end
qw=gt_rot(:,1); qx=gt_rot(:,2); qy=gt_rot(:,3); qz=gt_rot(:,4);
% +pi correction required: quaternion sign convention for this sensor mounting
% gives correct magnitude but wrong sign without it (same fix as myEKF.m)
theta_gt = wrapToPi_local(atan2(2*(qw.*qz+qx.*qy), 1-2*(qy.^2+qz.^2)) + pi);
dtheta   = [0; diff(theta_gt)] / dt_imu;   % heading rate rad/s
turn_mask = abs(dtheta) > 0.15;            % turning when |dθ/dt| > 0.15 rad/s

fprintf('\nTurning phases (|dθ/dt| > 0.15 rad/s):\n');
% find start/end of each turning phase
edges = diff([0; turn_mask; 0]);
turn_starts = find(edges == 1);
turn_ends   = find(edges == -1) - 1;
for i = 1:length(turn_starts)
    ts = t(min(turn_starts(i),n)); te = t(min(turn_ends(i),n));
    theta_change = rad2deg(theta_gt(min(turn_ends(i),n)) - theta_gt(min(turn_starts(i),n)));
    fprintf('  Turn %d: t=%.1f–%.1f s (%.1f s),  Δθ=%.1f deg\n', i, ts, te, te-ts, theta_change);
end

% -------------------------------------------------------------------------
% 3.  Heading: GT vs EKF theta
% -------------------------------------------------------------------------
theta_ekf = X_Est(1:n, 3);
theta_err = wrapToPi_local(theta_ekf - theta_gt(1:n));
fprintf('\n--- Heading error (GT vs EKF theta) ---\n');
fprintf('Max |theta error|    : %.2f deg\n', rad2deg(max(abs(theta_err))));
fprintf('RMS theta error      : %.2f deg\n', rad2deg(sqrt(mean(theta_err.^2))));
fprintf('Final theta error    : %.2f deg\n', rad2deg(theta_err(end)));

% Heading error timing diagnostics
theta_err_deg = rad2deg(theta_err);
exceed_90_idx  = find(abs(theta_err_deg) > 90,  1);
exceed_180_idx = find(abs(theta_err_deg) > 175, 1);
if ~isempty(exceed_90_idx)
    fprintf('First |heading error| > 90 deg  : t = %.2f s\n', t(exceed_90_idx));
else
    fprintf('First |heading error| > 90 deg  : never\n');
end
if ~isempty(exceed_180_idx)
    fprintf('First |heading error| > 175 deg : t = %.2f s\n', t(exceed_180_idx));
else
    fprintf('First |heading error| > 175 deg : never\n');
end

fprintf('\n--- Heading error at key time points ---\n');
fprintf('%-8s  %-14s  %-14s  %-14s\n', 'Time(s)', 'GT theta(deg)', 'EKF theta(deg)', 'Error(deg)');
for t_query = [10, 15, 20, 25, 30]
    [~, idx] = min(abs(t - t_query));
    if idx <= n
        fprintf('t=%-5.0f   %-14.2f  %-14.2f  %-14.2f\n', t_query, ...
            rad2deg(theta_gt(idx)), rad2deg(theta_ekf(idx)), theta_err_deg(idx));
    end
end
fprintf('(X(6) b_gyro time series requires separate myEKF diagnostic pass)\n');

% -------------------------------------------------------------------------
% 4.  Accelerometer during turns: detect gravity-leakage
% -------------------------------------------------------------------------
accel_bias_x = 9.84855;
accel_bias_y = 0.07485;
ax_corrected = accel_raw(:,1) - accel_bias_x;
ay_corrected = accel_raw(:,2) - accel_bias_y;

fprintf('\n--- Accelerometer: corrected values during turns ---\n');
if any(turn_mask)
    fprintf('  Mean |ax| during turns : %.4f m/s^2  (should be ~0)\n', mean(abs(ax_corrected(turn_mask))));
    fprintf('  Mean |ay| during turns : %.4f m/s^2  (should be ~0 if no lateral motion)\n', mean(abs(ay_corrected(turn_mask))));
    fprintf('  Max  |ax| during turns : %.4f m/s^2\n', max(abs(ax_corrected(turn_mask))));
    fprintf('  Max  |ay| during turns : %.4f m/s^2\n', max(abs(ay_corrected(turn_mask))));
    fprintf('  Mean |ax| NOT in turns : %.4f m/s^2\n', mean(abs(ax_corrected(~turn_mask))));
    fprintf('  Mean |ay| NOT in turns : %.4f m/s^2\n', mean(abs(ay_corrected(~turn_mask))));
end

% -------------------------------------------------------------------------
% 5.  ToF gating: count how many updates were ACCEPTED vs REJECTED
%     (Reconstruct manually from state data + sensor data)
% -------------------------------------------------------------------------
fprintf('\n--- ToF2 innovation analysis (reconstruct from EKF state) ---\n');
arena_y_max = 1.22; arena_y_min = -1.22;
arena_x_max = 1.22; arena_x_min = -1.22;
R_tof       = 0.01;
gamma       = 3.84;

tof2_data = squeeze(out.Sensor_ToF2.signals.values);
if size(tof2_data,1)==4, tof2_data=tof2_data'; end

% For each IMU step, compute what the ToF2 innovation WOULD be
h_x_vec     = zeros(N,1);
innov_vec   = zeros(N,1);
S_vec       = zeros(N,1);
maha_vec    = zeros(N,1);
tof2_raw_interp = interp1(out.Sensor_ToF2.time, tof2_data(:,1), imu_time, 'nearest', 'extrap');
P22 = squeeze(P_Est(2,2,:));

for k = 1:N
    theta_k = X_Est(k,3); y_k = X_Est(k,2); x_k = X_Est(k,1);
    phi2 = wrapToPi_local(theta_k + pi/2);   % ToF2 left direction

    % Compute expected distance (simplified: pick wall based on phi)
    [h_x_k, ~] = calc_tof(x_k, y_k, theta_k, pi/2, arena_x_max, arena_x_min, arena_y_max, arena_y_min);
    raw = tof2_raw_interp(k);
    innov = raw - h_x_k;
    S = P22(k) + R_tof;

    h_x_vec(k)   = h_x_k;
    innov_vec(k) = innov;
    S_vec(k)     = S;
    maha_vec(k)  = innov^2 / S;
end

pct_gated = 100 * mean(maha_vec > gamma);
fprintf('  Percentage of ToF2 steps GATED OUT : %.1f%%\n', pct_gated);
fprintf('  Max Mahalanobis score               : %.2f  (gate = %.2f)\n', max(maha_vec), gamma);
fprintf('  Median Mahalanobis score            : %.4f\n', median(maha_vec));

% Per-phase gating
if any(turn_mask)
    fprintf('  Gating DURING turns                 : %.1f%% rejected\n', 100*mean(maha_vec(turn_mask) > gamma));
    fprintf('  Gating OUTSIDE turns                : %.1f%% rejected\n', 100*mean(maha_vec(~turn_mask) > gamma));
end

% -------------------------------------------------------------------------
% 6.  PLOTS
% -------------------------------------------------------------------------
figure(1); clf;
plot(GT_rel(1:n,1), GT_rel(1:n,2), 'k-', 'LineWidth', 2, 'DisplayName', 'GT'); hold on;
plot(X_rel(1:n,1),  X_rel(1:n,2),  'b--','LineWidth', 1.5,'DisplayName', 'EKF');
xlabel('X (m)'); ylabel('Y (m)'); title('Task 2 Trajectory'); legend; grid on; axis equal;
sgtitle('Fig 1: Task 2 trajectory (should be a closed loop)');

figure(2); clf;
subplot(2,1,1);
plot(t(1:n), rad2deg(theta_gt(1:n)), 'k-', 'DisplayName','GT theta'); hold on;
plot(t(1:n), rad2deg(theta_ekf),      'b--','DisplayName','EKF theta');
ylabel('Heading (deg)'); title('GT vs EKF heading'); legend; grid on;
yl = ylim; % shade turning regions
for i = 1:length(turn_starts)
    ts_idx = min(turn_starts(i), n); te_idx = min(turn_ends(i), n);
    patch([t(ts_idx) t(te_idx) t(te_idx) t(ts_idx)], [yl(1) yl(1) yl(2) yl(2)], ...
          'r','FaceAlpha',0.15,'EdgeColor','none','HandleVisibility','off');
end
subplot(2,1,2);
plot(t(1:n), rad2deg(theta_err), 'm'); hold on; yline(0,'k--');
ylabel('Theta error (deg)'); xlabel('Time (s)'); title('Heading error (EKF - GT)'); grid on;
sgtitle('Fig 2: Heading tracking — red shading = turning phase');

figure(3); clf;
subplot(3,1,1);
plot(t, ax_corrected,'b'); hold on;
if any(turn_mask)
    plot(t(turn_mask), ax_corrected(turn_mask),'r.','MarkerSize',3);
end
yline(0,'k--'); ylabel('ax corrected (m/s^2)');
title('Accelerometer corrected (blue=all, red=during turns)'); grid on;
subplot(3,1,2);
plot(t, ay_corrected,'b'); hold on;
if any(turn_mask)
    plot(t(turn_mask), ay_corrected(turn_mask),'r.','MarkerSize',3);
end
yline(0,'k--'); ylabel('ay corrected (m/s^2)'); grid on;
subplot(3,1,3);
plot(t, X_Est(1:N,5),'r'); ylabel('vy\_est (m/s)'); xlabel('Time (s)');
title('Estimated lateral velocity'); grid on;
sgtitle('Fig 3: Accelerometer during turns (red dots)');

figure(4); clf;
subplot(2,1,1);
plot(t, innov_vec,'m'); hold on; yline(0,'k--');
xlabel('Time (s)'); ylabel('Innovation (m)');
title('ToF2 innovation over time'); grid on;
if any(turn_mask)
    yl = ylim;
    for i = 1:length(turn_starts)
        ts_idx = min(turn_starts(i),n); te_idx = min(turn_ends(i),n);
        patch([t(ts_idx) t(te_idx) t(te_idx) t(ts_idx)], [yl(1) yl(1) yl(2) yl(2)], ...
              'r','FaceAlpha',0.15,'EdgeColor','none','HandleVisibility','off');
    end
end
subplot(2,1,2);
semilogy(t, maha_vec,'b'); hold on;
yline(gamma,'r--','LineWidth',2,'DisplayName',sprintf('Gate (\\gamma=%.2f)',gamma));
xlabel('Time (s)'); ylabel('Mahalanobis score');
title('ToF2 gating  —  above red line = REJECTED');
legend; grid on;
if any(turn_mask)
    yl = ylim;
    for i = 1:length(turn_starts)
        ts_idx = min(turn_starts(i),n); te_idx = min(turn_ends(i),n);
        patch([t(ts_idx) t(te_idx) t(te_idx) t(ts_idx)], [yl(1) yl(1) yl(2) yl(2)], ...
              'r','FaceAlpha',0.15,'EdgeColor','none','HandleVisibility','off');
    end
end
sgtitle('Fig 4: ToF2 Mahalanobis gating — red shading = turning phases');

figure(5); clf;
plot(t(1:n), err_x,'b','DisplayName','Error X'); hold on;
plot(t(1:n), err_y,'r','DisplayName','Error Y');
yline(0,'k--');
xlabel('Time (s)'); ylabel('Position error (m)');
title('Position error over time (EKF - GT)'); legend; grid on;
if any(turn_mask)
    yl = ylim;
    for i = 1:length(turn_starts)
        ts_idx = min(turn_starts(i),n); te_idx = min(turn_ends(i),n);
        patch([t(ts_idx) t(te_idx) t(te_idx) t(ts_idx)], [yl(1) yl(1) yl(2) yl(2)], ...
              'r','FaceAlpha',0.15,'EdgeColor','none','HandleVisibility','off');
    end
end
sgtitle('Fig 5: Position error (red shading = turning phases) — look for error growth IN turns');

fprintf('\nDone. Check figures, especially:\n');
fprintf('  Fig 2: Does EKF theta track GT theta during turns?\n');
fprintf('  Fig 4: Are ToF2 updates being rejected during turns?\n');
fprintf('  Fig 5: Does error grow DURING turns (accel issue) or AFTER turns (heading issue)?\n');

%% -----------------------------------------------------------------------
function angle = wrapToPi_local(a)
    angle = mod(a + pi, 2*pi) - pi;
end

function [h_x, wall_idx] = calc_tof(x, y, theta, alpha, x_max, x_min, y_max, y_min)
    phi = mod(theta + alpha + pi, 2*pi) - pi;
    d_right = inf; d_left = inf; d_top = inf; d_bottom = inf;
    if cos(phi) >  1e-6, d_right  = (x_max - x) / cos(phi); end
    if cos(phi) < -1e-6, d_left   = (x_min - x) / cos(phi); end
    if sin(phi) >  1e-6, d_top    = (y_max - y) / sin(phi); end
    if sin(phi) < -1e-6, d_bottom = (y_min - y) / sin(phi); end
    [h_x, wall_idx] = min([d_right, d_left, d_top, d_bottom]);
end
