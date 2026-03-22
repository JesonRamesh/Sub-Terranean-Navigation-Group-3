%% verify_step5_Q.m — Session 0, Step 5: Process Noise Q Matrix Verification
% Run this from the Coursework folder in MATLAB.
% Output is printed only — no files are saved, no variables are modified in myEKF.m.
%
% Uses verified noise values from Steps 1 and 2 to derive principled Q values,
% then compares them to the current Q = diag([1e-4, 1e-4, 1e-4, 0.1, 0.1]) in myEKF.m.
%
% State vector in myEKF.m: [x; y; theta; vx_body; vy_body]

clc;
fprintf('=============================================================\n');
fprintf(' Session 0 — Step 5: Process Noise Q Matrix Verification\n');
fprintf('=============================================================\n\n');

% ------------------------------------------------------------------ %
%  Verified sensor noise values (from Steps 1 and 2)                  %
% ------------------------------------------------------------------ %
R_gyro_verified  = 1.275390e-06;   % (rad/s)^2  — Step 1c, axis 1, stationary std^2
R_accel_fwd      = 1.154904e-03;   % (m/s^2)^2  — Step 2e, forward axis (col 3) std^2
R_accel_lat      = 0.01462^2;      % (m/s^2)^2  — Step 2a, lateral axis (col 2) std = 0.01462

% Sample rate (from Steps 1 and 3 output — 200 Hz, NOT the 104 Hz assumed in session doc)
fs  = 200.0;          % Hz
dt  = 1 / fs;         % 0.005 s per IMU step

fprintf('Verified sensor noise inputs:\n');
fprintf('  R_gyro_verified  (axis 1 stationary var): %.6e (rad/s)^2\n', R_gyro_verified);
fprintf('  R_accel_fwd      (axis 3 stationary var): %.6e (m/s^2)^2\n', R_accel_fwd);
fprintf('  R_accel_lat      (axis 2 stationary var): %.6e (m/s^2)^2\n', R_accel_lat);
fprintf('  Sample rate (verified from data):         %.1f Hz\n', fs);
fprintf('  dt:                                       %.6f s\n', dt);
fprintf('\n');
fprintf('NOTE: The session doc assumed 104 Hz.  Actual rate is 200 Hz.\n');
fprintf('      dt = %.4f s  (not %.4f s as assumed)\n', dt, 1/104);
fprintf('      This affects all derived Q values by a factor of (%.4f/%.4f)^2 = %.4f\n', ...
    dt, 1/104, (dt / (1/104))^2);
fprintf('\n');

% ------------------------------------------------------------------ %
%  DERIVE Q VALUES FROM FIRST PRINCIPLES                              %
% ------------------------------------------------------------------ %
fprintf('--- Derivation of Q values from sensor noise ---\n\n');

% --- Q_theta ---
% theta_k = theta_{k-1} + omega * dt
% omega has noise variance R_gyro per reading.
% Noise added to theta per step: delta_theta_noise = noise_omega * dt
% Var(delta_theta) = R_gyro * dt^2
Q_theta_derived = R_gyro_verified * dt^2;

fprintf('Q_theta (heading):\n');
fprintf('  Formula:  Q_theta = R_gyro * dt^2\n');
fprintf('  = %.6e * (%.6f)^2\n', R_gyro_verified, dt);
fprintf('  = %.6e  rad^2\n', Q_theta_derived);
fprintf('  Current in myEKF.m: 1.000e-04  rad^2\n');
fprintf('  Ratio (current / derived): %.2e x\n', 1e-4 / Q_theta_derived);
fprintf('\n');

% --- Q_vx (body-frame forward velocity) ---
% vx_k = vx_{k-1} + ax * dt
% ax has noise variance R_accel_fwd per reading.
% Noise added to vx per step: delta_vx_noise = noise_ax * dt
% Var(delta_vx) = R_accel_fwd * dt^2
%
% NOTE: myEKF.m currently uses the VERTICAL axis (col 1) for ax — see Step 2 CRITICAL BUG.
% The CORRECT axis for forward velocity is column 3 (R_accel_fwd).
% After the bug is fixed, Q_vx should be derived from R_accel_fwd.
Q_vx_derived = R_accel_fwd * dt^2;

fprintf('Q_vx (body forward velocity) — using CORRECT axis (col 3) noise:\n');
fprintf('  Formula:  Q_vx = R_accel_fwd * dt^2\n');
fprintf('  = %.6e * (%.6f)^2\n', R_accel_fwd, dt);
fprintf('  = %.6e  (m/s)^2\n', Q_vx_derived);
fprintf('  Current in myEKF.m: 1.000e-01  (m/s)^2\n');
fprintf('  Ratio (current / derived): %.2e x\n', 0.1 / Q_vx_derived);
fprintf('\n');

% --- Q_vy (body-frame lateral velocity) ---
Q_vy_derived = R_accel_lat * dt^2;

fprintf('Q_vy (body lateral velocity) — axis col 2 (lateral, correct in myEKF.m):\n');
fprintf('  Formula:  Q_vy = R_accel_lat * dt^2\n');
fprintf('  = %.6e * (%.6f)^2\n', R_accel_lat, dt);
fprintf('  = %.6e  (m/s)^2\n', Q_vy_derived);
fprintf('  Current in myEKF.m: 1.000e-01  (m/s)^2\n');
fprintf('  Ratio (current / derived): %.2e x\n', 0.1 / Q_vy_derived);
fprintf('\n');

% --- Q_x, Q_y (position) ---
% Position is updated as: x_k = x_{k-1} + (vx*cos(th) - vy*sin(th)) * dt
% The process noise on position comes from uncertainty in velocity.
% Since velocity is tracked explicitly, Q_x should model only unaccounted
% position jumps (e.g. wheel slip teleporting the robot). Typically Q_x << Q_theta.
% A principled lower bound: integrate velocity noise over one step.
% If vx noise per step = sqrt(Q_vx_derived), then:
%   position noise per step ≈ sqrt(Q_vx_derived) * dt
Q_x_derived = Q_vx_derived * dt^2;   % lower bound — position error from velocity noise
Q_x_practical = 1e-6;                 % commonly used practical value

fprintf('Q_x = Q_y (position):\n');
fprintf('  Theoretical lower bound (from Q_vx * dt^2): %.6e  m^2\n', Q_x_derived);
fprintf('  Typical practical value:                    1.00e-06  m^2\n');
fprintf('  Current in myEKF.m:                         1.00e-04  m^2\n');
fprintf('  Current / lower bound ratio:                %.2e x\n', 1e-4 / Q_x_derived);
fprintf('  Current / practical ratio:                  %.2e x\n', 1e-4 / Q_x_practical);
fprintf('\n');

% ------------------------------------------------------------------ %
%  COMPARISON TABLE                                                   %
% ------------------------------------------------------------------ %
fprintf('--- Summary comparison table ---\n\n');
fprintf('%-10s  %-15s  %-15s  %-12s  %s\n', ...
    'State', 'Current Q', 'Derived Q', 'Ratio', 'Implication');
fprintf('%s\n', repmat('-', 1, 78));

entries = {
    'Q_x',     1e-4,           Q_x_derived,     'position';
    'Q_y',     1e-4,           Q_x_derived,     'position';
    'Q_theta', 1e-4,           Q_theta_derived, 'heading';
    'Q_vx',    0.1,            Q_vx_derived,    'fwd vel (after bug fix)';
    'Q_vy',    0.1,            Q_vy_derived,    'lat vel';
};

for i = 1:size(entries, 1)
    name    = entries{i,1};
    current = entries{i,2};
    derived = entries{i,3};
    label   = entries{i,4};
    ratio   = current / derived;
    fprintf('%-10s  %-15.3e  %-15.3e  %-12.2e  %s\n', ...
        name, current, derived, ratio, label);
end
fprintf('\n');

% ------------------------------------------------------------------ %
%  IMPLICATIONS                                                       %
% ------------------------------------------------------------------ %
fprintf('--- Implications ---\n\n');

fprintf('1. Q_theta is %.2e x larger than derived.\n', 1e-4 / Q_theta_derived);
fprintf('   Effect: filter assumes heading can drift %.4f rad (%.2f deg) per step\n', ...
    sqrt(1e-4), sqrt(1e-4) * 180/pi);
fprintf('   vs actual gyro noise contribution of %.4e rad per step.\n', sqrt(Q_theta_derived));
fprintf('   The inflated Q_theta allows the magnetometer/ToF to correct heading more,\n');
fprintf('   but since R_mag = 100 makes K_mag ≈ 0, gyro dominates heading regardless.\n\n');

fprintf('2. Q_vx is %.2e x larger than derived (using correct forward axis).\n', 0.1 / Q_vx_derived);
fprintf('   Effect: filter assumes velocity can jump %.4f m/s per step unpredictably.\n', sqrt(0.1));
fprintf('   At %g Hz this implies %.1f m/s^2 of unmodelled acceleration,\n', fs, sqrt(0.1)*fs);
fprintf('   far beyond the accel_clip = 2.0 m/s^2 guard.\n');
fprintf('   The large Q_vx means the covariance P grows fast, so ToF corrections\n');
fprintf('   dominate velocity/position updates. This may be intentional as a\n');
fprintf('   workaround for the broken accel axis, but causes velocity to drift freely.\n\n');

fprintf('3. CRITICAL INTERACTION: the Step 2 bug (wrong accel axis) means the\n');
fprintf('   current ax = accel_data(k,1) - 9.84855 has a residual of +0.154 m/s^2.\n');
fprintf('   This residual is NOT clipped (< 2.0 m/s^2 clip) and integrates into vx\n');
fprintf('   every step, giving a velocity drift of ~%.3f m/s per second of motion.\n', 0.15412);
fprintf('   Over a 30-second task this accumulates to ~%.1f m of position error.\n', ...
    0.5 * 0.15412 * 30);
fprintf('   Large Q_vx makes the ToF sensors absorb this but only while walls are visible.\n\n');

fprintf('4. After fixing the accel axis bug (Sessions 1-5), a tighter Q_vx\n');
fprintf('   closer to the derived value (%.2e) would be appropriate,\n', Q_vx_derived);
fprintf('   but some inflation is justified for Mecanum wheel slip (unmodelled dynamics).\n');
fprintf('   Suggested starting point: Q_vx = Q_vy = 1e-4 (still 10^4 x derived,\n');
fprintf('   leaving room for slip, but 1000 x tighter than current).\n\n');

fprintf('\n=============================================================\n');
fprintf(' Step 5 complete. Paste this output back to Claude.\n');
fprintf('=============================================================\n');
