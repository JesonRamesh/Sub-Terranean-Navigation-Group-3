function [X_Est, P_Est] = myEKF(acc, gyro, mag, ToF1, ToF2, ToF3, Temp, LP_acc)
% EKF for Sub-Terranean Navigation — Simulink MATLAB Function Block
% Called once per timestep (200Hz, dt=0.005s)
% Inputs (per timestep):
%   acc    : 3x1 accelerometer [ax; ay; az] m/s²
%   gyro   : 3x1 gyroscope     [gx; gy; gz] rad/s  (gx = yaw axis)
%   mag    : 3x1 magnetometer  [mx; my; mz] Tesla
%   ToF1   : 4x1 [dist; ?; ?; status]  right-facing  (alpha=-pi/2)
%   ToF2   : 4x1 [dist; ?; ?; status]  forward-facing (alpha=0)
%   ToF3   : 4x1 [dist; ?; ?; status]  left-facing   (alpha=+pi/2)
%   Temp   : unused (template placeholder)
%   LP_acc : unused (template placeholder)
% Outputs:
%   X_Est  : 1x5 [x, y, theta, vx_world, vy_world]
%   P_Est  : 5x5 covariance (top-left block)

%% Persistent state — survives between Simulink timestep calls
persistent X P initialized call_count

% Calibration (computed during startup)
persistent gyro_bias accel_bias_fwd accel_bias_lat
persistent x0 y0 mag_global_offset
persistent ab_xmax ab_xmin ab_ymax ab_ymin

% Mag anomaly detection
persistent theta_gyro_ref gyro_ref_bias mag_disable_until
persistent mag_innov_buf mag_innov_count mag_norm_ref
persistent gyro_mag_diverge_count prev_innovation_mag mag_innov_growing_count

% ToF rate tracking
persistent prev_tof1_dist prev_tof1_genuine_time
persistent prev_tof2_dist prev_tof2_genuine_time
persistent prev_tof3_dist prev_tof3_genuine_time

% Startup accumulation buffers
persistent startup_gyro startup_accel startup_tof1 startup_tof2 startup_tof3
persistent startup_mag startup_done startup_buf_count

persistent zupt_accel_fwd_buf zupt_accel_lat_buf zupt_gyro_buf zupt_buf_count

persistent post_turn_steps

%% Constants
dt              = 0.005;
alpha_tof1      = -pi/2;
alpha_tof2      =  0;
alpha_tof3      =  pi/2;
mag_hard_iron   = [-3.705e-05; 4.415e-05];           % Now a 2x1 column vector
mag_soft_matrix = [1.0958, 0.0120; 0.0120, 0.9196];  % New 2x2 matrix
theta0          = pi/2;
R_mag           = 4.0;
R_tof           = 0.01;
gamma_threshold = 6.0;

%% First call: initialise all persistent variables
if isempty(initialized)
    call_count   = 0;
    startup_done = false;
    startup_buf_count = 0;

    % Startup buffers — pre-allocate for up to 500 samples
    startup_gyro  = zeros(500, 3);
    startup_accel = zeros(500, 3);
    startup_tof1  = zeros(500, 4);
    startup_tof2  = zeros(500, 4);
    startup_tof3  = zeros(500, 4);
    startup_mag   = zeros(500, 3);

    % Fallback calibration constants (used if startup fails)
    gyro_bias      =  0.00186;
    accel_bias_fwd = -0.396;
    accel_bias_lat =  0.07485;
    x0             =  0.0;
    y0             = -0.93;

    % ZUPT
    zupt_accel_fwd_buf = zeros(40, 1);
    zupt_accel_lat_buf = zeros(40, 1);
    zupt_gyro_buf      = zeros(40, 1);
    zupt_buf_count     = 0;

    % arena_bounds = struct('x_max',  1.22, 'x_min', -1.22, ...
    %                       'y_max',  1.22, 'y_min', -1.22);
    ab_xmax =  1.22; ab_xmin = -1.22;
    ab_ymax =  1.22; ab_ymin = -1.22;

    % These will be properly set after startup completes
    mag_global_offset       = 0;
    theta_gyro_ref          = theta0;
    gyro_ref_bias           = gyro_bias;
    mag_disable_until       = 0;
    mag_innov_buf           = zeros(20, 1);
    mag_innov_count         = 0;
    gyro_mag_diverge_count  = 0;
    prev_innovation_mag     = 0;
    mag_innov_growing_count = 0;
    mag_norm_ref            = 1.0; 

    prev_tof1_dist         = NaN;
    prev_tof1_genuine_time = NaN;
    prev_tof2_dist         = NaN;
    prev_tof2_genuine_time = NaN;
    prev_tof3_dist         = NaN;
    prev_tof3_genuine_time = NaN;

    % State and covariance — will be reset after startup
    X = [0; -0.93; theta0; 0; 0; gyro_bias];
    P = diag([0.1, 0.1, 0.1, 2.0, 2.0, 0.25]);

    post_turn_steps = 0;

    initialized = true;
end

call_count = call_count + 1;
current_time = (call_count - 1) * dt;

%% Startup phase: accumulate samples, compute calibration
if ~startup_done
    startup_buf_count = startup_buf_count + 1;
    n = min(startup_buf_count, 500);
    startup_gyro(n, :)  = reshape(gyro,  1, 3);
    startup_accel(n, :) = reshape(acc,   1, 3);
    startup_mag(n, :)   = reshape(mag,   1, 3);
    startup_tof1(n, :)  = reshape(ToF1,  1, 4);
    startup_tof2(n, :)  = reshape(ToF2,  1, 4);
    startup_tof3(n, :)  = reshape(ToF3,  1, 4);

    % Try to find 100 consecutive stationary samples once we have enough
    if startup_buf_count >= 100
        stat_start   = 0;
        consec_count = 0;
        search_limit = min(startup_buf_count, 500);
        for i = 1:search_limit
            if abs(startup_gyro(i, 1)) < 0.01
                consec_count = consec_count + 1;
                if consec_count == 100
                    stat_start = i - 99;
                    break;
                end
            else
                consec_count = 0;
            end
        end

        if stat_start > 0
            n_stat     = min(200, search_limit - stat_start + 1);
            stat_range = stat_start : (stat_start + n_stat - 1);

            % Per-run calibration
            gyro_bias      = mean(startup_gyro(stat_range, 1));
            accel_bias_fwd = mean(startup_accel(stat_range, 3));
            accel_bias_lat = mean(startup_accel(stat_range, 2));

            % Arena wall + x0/y0 estimation from ToF
            tof1_in = startup_tof1(stat_range, 1);
            tof2_in = startup_tof2(stat_range, 1);
            tof3_in = startup_tof3(stat_range, 1);

            gen1 = tof1_in([true; diff(tof1_in) ~= 0]);
            gen2 = tof2_in([true; diff(tof2_in) ~= 0]);
            gen3 = tof3_in([true; diff(tof3_in) ~= 0]);
            gen1 = gen1(gen1 > 0);
            gen2 = gen2(gen2 > 0);
            gen3 = gen3(gen3 > 0);

            if ~isempty(gen1) && ~isempty(gen2) && ~isempty(gen3)
                s1 = sort(gen1); d_rgt = s1(max(1,floor(end/2)));
                s2 = sort(gen2); d_fwd = s2(max(1,floor(end/2)));
                s3 = sort(gen3); d_lft = s3(max(1,floor(end/2)));

                if d_rgt > 0 && d_fwd > 0 && d_lft > 0 && ...
                   ~isnan(d_rgt) && ~isnan(d_fwd) && ~isnan(d_lft)

                    % Estimate starting position from wall distances
                    % At theta=pi/2: right wall is +X, forward wall is +Y
                    wall_y_max =  1.22;   % known arena dimension
                    wall_x_max =  1.22;
                    x0 = wall_x_max - d_rgt;
                    y0 = wall_y_max - d_fwd;

                    wall_x_min = x0 - d_lft;
                    wall_y_min = -wall_y_max;

                    ab_xmax = wall_x_max;
                    ab_xmin = wall_x_min;
                    ab_ymax = wall_y_max;
                    ab_ymin = wall_y_min;
                end
            end

            % 1. Initial offset calculation
            mag_raw0   = [startup_mag(1,2); startup_mag(1,3)]; % This is 2x1
            mag_cal0   = mag_soft_matrix * (mag_raw0 - mag_hard_iron); % No ' needed here if hard_iron is 2x1
            z_mag0     = atan2(mag_cal0(2), mag_cal0(1));
            mag_global_offset = wrapToPi_fn(theta0 - z_mag0);
            
            % 2. Norm reference (The error line)
            mag_stat_raw = startup_mag(stat_range, 2:3)'; % This is 2 x N
            % We subtract hard_iron from every column of the buffer
            mag_stat_cal = mag_soft_matrix * (mag_stat_raw - repmat(mag_hard_iron, 1, size(mag_stat_raw, 2)));
            mag_norm_ref = mean(mag_stat_cal(1,:).^2 + mag_stat_cal(2,:).^2);

            % Initialise filter state with estimated x0, y0
            X = [x0; y0; theta0; 0; 0; gyro_bias];
            P = diag([0.1, 0.1, 0.1, 2.0, 2.0, 0.25]);

            theta_gyro_ref = theta0;
            gyro_ref_bias  = gyro_bias;

            startup_done = true;
        end
    end

    % During startup, return zero estimates
    X_Est = [0; -0.93; pi/2; 0; 0; 0; 0; 0];   % 1x8
    P_Est = zeros(5, 5);
    return;
end

%% Extract current sensor values
gx      = gyro(1);          % yaw axis gyro (rad/s)
ax_body_raw = acc(3);       % forward accel (m/s²)
ay_body_raw = acc(2);       % lateral accel (m/s²)

tof1_dist   = ToF1(1);
tof1_status = ToF1(4);
tof2_dist   = ToF2(1);
tof2_status = ToF2(4);
tof3_dist   = ToF3(1);
tof3_status = ToF3(4);

mag_y = mag(2);
mag_z = mag(3);

%% Prediction step
omega   = gx - X(6);
% [x, y, theta, vx, vy, gyro_bias]
Q_base = diag([ ...
    1e-6, ...   % x (Position doesn't change much on its own)
    1e-6, ...   % y
    2.5e-4, ...   % theta (Tighter heading makes it rely more on the Gyro)
    0.05, ...   % vx (Significant reduction from 0.5 to prevent "ghost" velocity)
    0.05, ...   % vy
    1e-6  ...   % gyro_bias (Make bias very stable)
]) * dt;

ax_body = max(-2.0, min(2.0, ax_body_raw - accel_bias_fwd));
ay_body = max(-2.0, min(2.0, ay_body_raw - accel_bias_lat));

ax_world = ax_body * cos(X(3)) - ay_body * sin(X(3));
ay_world = ax_body * sin(X(3)) + ay_body * cos(X(3));

%vel_damp = 1.0 - 2.0 * dt;
% Detect turn: high omega indicates active rotation
if abs(omega) > 0.3
    post_turn_steps = 150;  % flag for 0.75s after turn ends
elseif post_turn_steps > 0
    post_turn_steps = post_turn_steps - 1;
end

% Aggressive damping only immediately after turns
% During straight motion: light damping to preserve forward velocity
if post_turn_steps > 0
    vel_damp = 1.0 - 2.0 * dt;   % aggressive: kill bad post-turn velocities
else
    vel_damp = 1.0;   % no damping: preserve valid straight-leg velocity
end

X_pred    = zeros(6, 1);
X_pred(1) = X(1) + X(4) * dt;
X_pred(2) = X(2) + X(5) * dt;
X_pred(3) = wrapToPi_fn(X(3) + omega * dt);
X_pred(4) = X(4) * vel_damp + ax_world * dt;
X_pred(5) = X(5) * vel_damp + ay_world * dt;
X_pred(6) = X(6);

F       = eye(6);
F(1, 4) = dt;
F(2, 5) = dt;
F(3, 6) = -dt;
F(4, 3) = (-ax_body * sin(X(3)) - ay_body * cos(X(3))) * dt;
F(5, 3) = ( ax_body * cos(X(3)) - ay_body * sin(X(3))) * dt;
F(4, 4) = vel_damp;
F(5, 5) = vel_damp;

% Adaptive Q adjustment
Q_k = Q_base;

% If we are turning, we EXPECT the heading to change, so we loosen Q(3,3)
if abs(omega) > 0.15
    Q_k(3,3) = Q_k(3,3) * 5.0; 
end

% If Mag is disabled, TIGHTEN heading noise.
% This prevents ToF noise from kicking the heading while the Mag is gone.
if call_count <= mag_disable_until
    Q_k(3,3) = Q_k(3,3) * 0.1; % Make heading 10x "heavier"
    Q_k(4,4) = Q_k(4,4) * 0.5; % Don't let velocity wander
    Q_k(5,5) = Q_k(5,5) * 0.5;
end
speed = sqrt(X(4)^2 + X(5)^2);
if speed > 0.05
    Q_k(6,6) = Q_k(6,6) * (1.0 + 5.0 * speed);
end

P_pred = F * P * F' + Q_k;
X      = X_pred;
P      = P_pred;

% Covariance Capping (Safety Guard)
P(1,1) = min(P(1,1), 0.2);  % Cap X uncertainty
P(2,2) = min(P(2,2), 0.2);  % Cap Y uncertainty
P(3,3) = min(P(3,3), 0.02); % Cap Heading uncertainty (approx 8 degrees)

%% Gyro reference update
%theta_gyro_ref = wrapToPi_fn(theta_gyro_ref + (gx - gyro_ref_bias) * dt);
theta_gyro_ref = wrapToPi_fn(theta_gyro_ref + (gx - X(6)) * dt);

%% Magnetometer update with anomaly detection
mag_raw   = [mag_y; mag_z];
mag_cal   = mag_soft_matrix * (mag_raw - mag_hard_iron'); % Note the transpose ' if hard_iron is 1x2
mag_y_cal = mag_cal(1);
mag_z_cal = mag_cal(2);

current_norm2 = mag_y_cal^2 + mag_z_cal^2;
norm_error_ratio = abs(current_norm2 - mag_norm_ref) / mag_norm_ref;

% If the field strength changes by more than 20%, it's guaranteed garbage.
% Keep refreshing the lockout until the field normalizes.
if norm_error_ratio > 0.15
    mag_disable_until = call_count + 100; % 0.5 second rolling lockout
end

z_mag = wrapToPi_fn(atan2(mag_z_cal, mag_y_cal) + mag_global_offset);
innovation_mag = wrapToPi_fn(z_mag - X(3));

% Criterion 1: innovation buffer
mag_innov_count = mag_innov_count + 1;
buf_idx = mod(mag_innov_count - 1, 20) + 1;
mag_innov_buf(buf_idx) = innovation_mag;
if mag_innov_count >= 20
    ib_mean = sum(mag_innov_buf) / 20;
    innov_var = sum((mag_innov_buf - ib_mean).^2) / 19;
    innov_rms = sqrt(sum(mag_innov_buf.^2) / 20);
    if innov_var > 0.04 || innov_rms > 0.15
        mag_disable_until = call_count + 400;
    end
end

% Criterion 2: innovation growing for 5 consecutive steps
if abs(innovation_mag) > abs(prev_innovation_mag) + 0.02
    mag_innov_growing_count = mag_innov_growing_count + 1;
    if mag_innov_growing_count >= 5
        mag_disable_until = call_count + 400;
        mag_innov_growing_count = 0;
    end
else
    mag_innov_growing_count = max(0, mag_innov_growing_count - 1);
end
prev_innovation_mag = innovation_mag;

% Criterion 3: sustained divergence from gyro reference
% gyro_mag_diff = abs(wrapToPi_fn(z_mag - X(3)));
gyro_mag_diff = abs(wrapToPi_fn(z_mag - theta_gyro_ref));
max_expected  = 0.008 * current_time + 0.12;
if gyro_mag_diff > max_expected
    gyro_mag_diverge_count = gyro_mag_diverge_count + 1;
    if gyro_mag_diverge_count >= 30
        mag_disable_until      = call_count + 400;
        gyro_mag_diverge_count = 0;
    end
else
    % Only decrement if well within tolerance (hysteresis)
    if gyro_mag_diff < max_expected * 0.5
        gyro_mag_diverge_count = max(0, gyro_mag_diverge_count - 1);
    end
end

% Apply mag update only when not disabled
if call_count > mag_disable_until
    % Use tighter R_mag when mag is confirmed clean (early in run)
    % if current_time < 8.0 && mag_innov_count >= 5
    %     R_mag_k = 1.0;   % Trust mag strongly in first 8 seconds
    % else
    %     R_mag_k = 4.0;   % Normal trust after that
    % end
    
    % If just recovered from an anomaly, don't trust the mag blindly yet.
    time_since_anomaly = call_count - mag_disable_until; 
    
    % Smoothly transition R_mag back to normal over 1 second (200 steps) after an anomaly clears.
    R_mag_k = 4.0;
    if time_since_anomaly < 200
        R_mag_k = 4.0 + 10.0 * (1 - time_since_anomaly/200);
    end

    H_mag = [0 0 1 0 0 0];
    S_mag = H_mag * P * H_mag' + R_mag_k;
    if innovation_mag^2 / S_mag < gamma_threshold
        K_mag = P * H_mag' / S_mag;
        X     = X + K_mag * innovation_mag;
        X(3)  = wrapToPi_fn(X(3));
        IKH   = eye(6) - K_mag * H_mag;
        P     = IKH * P * IKH' + K_mag * R_mag_k * K_mag';
        gyro_ref_bias = 0.95 * gyro_ref_bias + 0.05 * X(6);
    end
end

%% ToF distance + rate updates
% Helper: run one ToF sensor update
% ToF 1
if tof1_status == 0
    [h_x, H_tof] = calc_tof(X, ab_xmax, ab_xmin, ab_ymax, ab_ymin, alpha_tof1);
    S          = H_tof * P * H_tof' + R_tof;
    innovation = tof1_dist - h_x;
    if (innovation^2) / S < gamma_threshold
        K    = P * H_tof' / S;
        X    = X + K * innovation;
        X(3) = wrapToPi_fn(X(3));
        IKH  = eye(6) - K * H_tof;
        P    = IKH * P * IKH' + K * R_tof * K';
    end
    tof1_is_new = isnan(prev_tof1_dist) || (tof1_dist ~= prev_tof1_dist);
    if tof1_is_new && ~isnan(prev_tof1_dist) && call_count <= mag_disable_until
        dt_gen = current_time - prev_tof1_genuine_time;
        if dt_gen > 0.08
            spd = sqrt(X(4)^2 + X(5)^2);
            if spd > 0.05 && abs(omega) < 0.2
                phi        = wrapToPi_fn(X(3) + alpha_tof1);
                dro        = (tof1_dist - prev_tof1_dist) / dt_gen;
                dre        = -(X(4)*cos(phi) + X(5)*sin(phi));
                Hr         = zeros(1,6);
                Hr(3)      = X(4)*sin(phi) - X(5)*cos(phi);
                Hr(4)      = -cos(phi);
                Hr(5)      = -sin(phi);
                Sr         = Hr * P * Hr' + 0.5;
                ir         = dro - dre;
                if ir^2 < 9.0 * Sr
                    Kr   = P * Hr' / Sr;
                    X    = X + Kr * ir;
                    X(3) = wrapToPi_fn(X(3));
                    P    = (eye(6) - Kr * Hr) * P;
                end
            end
        end
    end
    if tof1_is_new
        prev_tof1_dist         = tof1_dist;
        prev_tof1_genuine_time = current_time;
    end
end

% ToF 2
if tof2_status == 0
    [h_x, H_tof] = calc_tof(X, ab_xmax, ab_xmin, ab_ymax, ab_ymin, alpha_tof2);
    S          = H_tof * P * H_tof' + R_tof;
    innovation = tof2_dist - h_x;
    if (innovation^2) / S < gamma_threshold
        K    = P * H_tof' / S;
        X    = X + K * innovation;
        X(3) = wrapToPi_fn(X(3));
        IKH  = eye(6) - K * H_tof;
        P    = IKH * P * IKH' + K * R_tof * K';
    end
    tof2_is_new = isnan(prev_tof2_dist) || (tof2_dist ~= prev_tof2_dist);
    if tof2_is_new && ~isnan(prev_tof2_dist) && call_count <= mag_disable_until
        dt_gen = current_time - prev_tof2_genuine_time;
        if dt_gen > 0.08
            spd = sqrt(X(4)^2 + X(5)^2);
            if spd > 0.05 && abs(omega) < 0.2
                phi        = wrapToPi_fn(X(3) + alpha_tof2);
                dro        = (tof2_dist - prev_tof2_dist) / dt_gen;
                dre        = -(X(4)*cos(phi) + X(5)*sin(phi));
                Hr         = zeros(1,6);
                Hr(3)      = X(4)*sin(phi) - X(5)*cos(phi);
                Hr(4)      = -cos(phi);
                Hr(5)      = -sin(phi);
                Sr         = Hr * P * Hr' + 0.5;
                ir         = dro - dre;
                if ir^2 < 9.0 * Sr
                    Kr   = P * Hr' / Sr;
                    X    = X + Kr * ir;
                    X(3) = wrapToPi_fn(X(3));
                    P    = (eye(6) - Kr * Hr) * P;
                end
            end
        end
    end
    if tof2_is_new
        prev_tof2_dist         = tof2_dist;
        prev_tof2_genuine_time = current_time;
    end
end

% ToF 3
if tof3_status == 0
    [h_x, H_tof] = calc_tof(X, ab_xmax, ab_xmin, ab_ymax, ab_ymin, alpha_tof3);
    S          = H_tof * P * H_tof' + R_tof;
    innovation = tof3_dist - h_x;
    if (innovation^2) / S < gamma_threshold
        K    = P * H_tof' / S;
        X    = X + K * innovation;
        X(3) = wrapToPi_fn(X(3));
        IKH  = eye(6) - K * H_tof;
        P    = IKH * P * IKH' + K * R_tof * K';
    end
    tof3_is_new = isnan(prev_tof3_dist) || (tof3_dist ~= prev_tof3_dist);
    if tof3_is_new && ~isnan(prev_tof3_dist) && call_count <= mag_disable_until
        dt_gen = current_time - prev_tof3_genuine_time;
        if dt_gen > 0.08
            spd = sqrt(X(4)^2 + X(5)^2);
            if spd > 0.05 && abs(omega) < 0.2
                phi        = wrapToPi_fn(X(3) + alpha_tof3);
                dro        = (tof3_dist - prev_tof3_dist) / dt_gen;
                dre        = -(X(4)*cos(phi) + X(5)*sin(phi));
                Hr         = zeros(1,6);
                Hr(3)      = X(4)*sin(phi) - X(5)*cos(phi);
                Hr(4)      = -cos(phi);
                Hr(5)      = -sin(phi);
                Sr         = Hr * P * Hr' + 0.5;
                ir         = dro - dre;
                if ir^2 < 9.0 * Sr
                    Kr   = P * Hr' / Sr;
                    X    = X + Kr * ir;
                    X(3) = wrapToPi_fn(X(3));
                    P    = (eye(6) - Kr * Hr) * P;
                end
            end
        end
    end
    if tof3_is_new
        prev_tof3_dist         = tof3_dist;
        prev_tof3_genuine_time = current_time;
    end
end

%% Position consistency check — detect and recover from large position error
% If all three ToF sensors have large innovations simultaneously,
% the position estimate is likely badly wrong — accept stronger corrections
if tof1_status == 0 && tof2_status == 0 && tof3_status == 0
    [h1,~] = calc_tof(X, ab_xmax, ab_xmin, ab_ymax, ab_ymin, alpha_tof1);
    [h2,~] = calc_tof(X, ab_xmax, ab_xmin, ab_ymax, ab_ymin, alpha_tof2);
    [h3,~] = calc_tof(X, ab_xmax, ab_xmin, ab_ymax, ab_ymin, alpha_tof3);
    inn1 = abs(tof1_dist - h1);
    inn2 = abs(tof2_dist - h2);
    inn3 = abs(tof3_dist - h3);
    % All three sensors disagree by more than 0.3m — position is wrong
    if inn1 > 0.3 && inn2 > 0.3 && inn3 > 0.3
        % Increase position uncertainty to accept stronger corrections
        P(1,1) = max(P(1,1), 0.5);
        P(2,2) = max(P(2,2), 0.5);
    end
    % if inn1 > 0.2 && inn2 > 0.2 && inn3 > 0.2
    %     P(1,1) = max(P(1,1), 1.0);
    %     P(2,2) = max(P(2,2), 1.0);
    % end
end

%% ZUPT
% if call_count >= 40 && abs(omega) < 0.02 && ...
%    abs(ax_body) < 0.03 && abs(ay_body) < 0.03 && ...
%    abs(gx - X(6)) < 0.005
% 
%     % Part 1: Velocity ZUPT
%     H_v     = [0 0 0 1 0 0; 0 0 0 0 1 0];
%     innov_v = -[X(4); X(5)];
%     S_v     = H_v * P * H_v' + 0.002 * eye(2);
%     K_v     = P * H_v' / S_v;
%     X       = X + K_v * innov_v;
%     P       = (eye(6) - K_v * H_v) * P;
% 
%     % Part 2: Gyro bias ZUPT
%     H_b     = [0 0 0 0 0 1];
%     innov_b = gx - X(6);
%     S_b     = P(6,6) + 0.00005;
%     K_b     = P * H_b' / S_b;
%     X       = X + K_b * innov_b;
%     P       = (eye(6) - K_b * H_b) * P;
%     gyro_ref_bias = X(6);
% 
%     % Part 3: Soft heading ZUPT
%     H_h     = [0 0 1 0 0 0];
%     innov_h = -(omega * dt);
%     S_h     = H_h * P * H_h' + 0.005;
%     K_h     = P * H_h' / S_h;
%     X       = X + K_h * innov_h;
%     X(3)    = wrapToPi_fn(X(3));
%     P       = (eye(6) - K_h * H_h) * P;
% end
%% ZUPT — 40-sample variance window
zupt_buf_count = zupt_buf_count + 1;
buf_i = mod(zupt_buf_count - 1, 40) + 1;
zupt_accel_fwd_buf(buf_i) = ax_body;
zupt_accel_lat_buf(buf_i) = ay_body;
zupt_gyro_buf(buf_i)      = gx;

if zupt_buf_count >= 40
    az_mean  = sum(zupt_accel_fwd_buf) / 40;
    ay_mean  = sum(zupt_accel_lat_buf) / 40;
    gy_mean  = sum(zupt_gyro_buf) / 40;
    az_var   = sum((zupt_accel_fwd_buf - az_mean).^2) / 39;
    ay_var   = sum((zupt_accel_lat_buf - ay_mean).^2) / 39;
    gyro_var = sum((zupt_gyro_buf - gy_mean).^2) / 39;

    if az_var < 0.005 && ay_var < 0.005 && gyro_var < 0.0005
        % Part 1: Velocity ZUPT
        H_v     = [0 0 0 1 0 0; 0 0 0 0 1 0];
        innov_v = -[X(4); X(5)];
        S_v     = H_v * P * H_v' + 0.002 * eye(2);
        K_v     = P * H_v' / S_v;
        X       = X + K_v * innov_v;
        P       = (eye(6) - K_v * H_v) * P;

        % Part 2: Gyro bias ZUPT
        H_b     = [0 0 0 0 0 1];
        innov_b = gx - X(6);
        S_b     = P(6,6) + 0.00005;
        K_b     = P * H_b' / S_b;
        X       = X + K_b * innov_b;
        P       = (eye(6) - K_b * H_b) * P;
        gyro_ref_bias = X(6);

        % Part 3: Soft heading ZUPT
        H_h     = [0 0 1 0 0 0];
        innov_h = -(omega * dt);
        S_h     = H_h * P * H_h' + 0.005;
        K_h     = P * H_h' / S_h;
        X       = X + K_h * innov_h;
        X(3)    = wrapToPi_fn(X(3));
        P       = (eye(6) - K_h * H_h) * P;
    end
end

%% Slow-motion gyro bias refinement
% When nearly stationary (not a full stop but very slow),
% use current gyro reading as a soft bias measurement.
% Only fires when: slow speed AND low turn rate AND mag is reliable
if speed < 0.08 && abs(omega) < 0.03 && call_count > mag_disable_until
    H_b_soft    = [0 0 0 0 0 1];
    innov_b_soft = gx - X(6);
    S_b_soft    = P(6,6) + 0.002;
    K_b_soft    = P * H_b_soft' / S_b_soft;
    X           = X + K_b_soft * innov_b_soft;
    P           = (eye(6) - K_b_soft * H_b_soft) * P;
end

%% Output current state estimate
X_Est = [X(1:5); 0; 0; 0];    % 1x8 — padded to match model expectation
P_Est = P(1:5, 1:5);           % 5x5

end  % function myEKF

%% Local helper: ToF measurement model
function [h_x, H_tof] = calc_tof(X, xmax, xmin, ymax, ymin, alpha)
    x   = X(1);
    y   = X(2);
    phi = mod(X(3) + alpha + pi, 2*pi) - pi;

    d_right  = inf; d_left = inf; d_top = inf; d_bottom = inf;

    if cos(phi) > 1e-6
        d_right = (xmax - x) / cos(phi);
    elseif cos(phi) < -1e-6
        d_left  = (xmin - x) / cos(phi);
    end
    if sin(phi) > 1e-6
        d_top    = (ymax - y) / sin(phi);
    elseif sin(phi) < -1e-6
        d_bottom = (ymin - y) / sin(phi);
    end

    distances = [d_right, d_left, d_top, d_bottom];
    distances(distances <= 0) = inf;
    [h_x, wall_idx] = min(distances);

    if isinf(h_x)
        H_tof = zeros(1, length(X));
        return;
    end

    H_tof = zeros(1, 6);
    if wall_idx == 1 || wall_idx == 2
        H_tof(1) = -1 / cos(phi);
        if abs(tan(phi)) < 0.7 % Only trust if hitting wall nearly straight-on
            H_tof(3) = h_x * tan(phi) * 0.05; 
        end
    else
        H_tof(2) = -1 / sin(phi);
        if abs(cos(phi)/sin(phi)) < 0.7
            H_tof(3) = -h_x * (cos(phi) / sin(phi)) * 0.05;
        end
    end
end

%% Local helper: angle wrapping
function a = wrapToPi_fn(a)
    a = mod(a + pi, 2*pi) - pi;
end