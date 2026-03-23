function [X_Est, P_Est, GT] = myEKF(out)
% myEKF  Extended Kalman Filter for subterranean robot navigation.
%
%   [X_Est, P_Est, GT] = myEKF(out)
%
%   State vector: [px; py; vx; vy; theta]  (5 states)

%% ======================================================================
%  TUNING PARAMETERS — All adjustable values are here
%  ======================================================================

% --- Gyro ---
GYRO_YAW_ROW = 1;
GYRO_BIAS_WINDOW_SEC = 4.0;   % seconds at start to average for online bias

% --- Accel bias ---
ACCEL_BIAS = [10.003; 0.086; -0.376];

% --- Accel usage ---
ACCEL_FWD_ROW    = 2;
ACCEL_RIGHT_ROW  = 3;
ACCEL_FWD_SIGN   = 1;
ACCEL_RIGHT_SIGN = 1;
%  *** SET TO 0 TO DISABLE ACCEL ENTIRELY ***
ACCEL_SCALE = 0.0;

% --- Magnetometer (disabled — too unreliable) ---
MAG_ENABLE = false;
MAG_HARD_IRON = [4.45e-06; -4.84e-05; 8.90e-05];
R_MAG = 2.0;
MAG_UPDATE_INTERVAL = 10;

% --- Frame alignment ---
YAW_OFFSET = deg2rad(1.72);

% --- Arena ---
WALL_X_POS =  1.22;
WALL_X_NEG = -1.22;
WALL_Y_POS =  1.22;
WALL_Y_NEG = -1.22;

% --- ToF mounting: CONFIRMED left/back/right ---
TOF_ANGLES = [pi/2, pi, -pi/2];

% --- Process noise ---
Q_POS   = 1e-4;
Q_VEL   = 5.0;       % High — we don't trust accel-based velocity
Q_THETA = 5e-3;      % HIGH — let yaw uncertainty grow so ToF can correct it

% --- Velocity damping ---
VEL_DAMPING = 0.95;

% --- ToF measurement noise ---
R_TOF = 0.005;       % Trust ToF very strongly

% --- Mahalanobis gate ---
MAHAL_GATE = 15.0;   % Relaxed — allow larger innovations through

% --- ToF validity ---
TOF_MIN = 0.03;
TOF_MAX = 2.50;

% --- ZUPT ---
%  *** CHANGE THIS VALUE TO TUNE ZUPT SENSITIVITY ***
ZUPT_THRESHOLD = 0.10;
ZUPT_R = 1e-3;
ZUPT_WINDOW = 20;

% --- Arena bounds ---
ARENA_BOUND_ENABLE = true;
ARENA_MARGIN = 0.05;
R_BOUND = 0.001;

%% ======================================================================
%  EXTRACT DATA
%  ======================================================================
[gyro, accel, mag, tof1, tof2, tof3, sensor_time, gt_pos, gt_rot] = ...
    extractData(out);

N = length(sensor_time);

%% ======================================================================
%  GROUND TRUTH — yaw from quaternion
%  ======================================================================
gt_quat = gt_rot;
gt_yaw = atan2(2*(gt_quat(:,1).*gt_quat(:,4) + gt_quat(:,2).*gt_quat(:,3)), ...
               1 - 2*(gt_quat(:,3).^2 + gt_quat(:,4).^2));

GT = gt_pos;

%% ======================================================================
%  FIND FIRST VALID GT INDEX
%  ======================================================================
%  Zero-padded samples have quaternion [0,0,0,0]. Real data has unit
%  quaternion with norm ≈ 1. Find first sample with valid quaternion.

init_idx = 5;  % Minimum starting point
for i = 2:N
    qnorm = norm(gt_rot(i,:));
    if qnorm > 0.9 && qnorm < 1.1  % Valid unit quaternion
        init_idx = max(i, 5);
        break;
    end
end

% Average a small window of GT for clean init
init_window = min(20, N - init_idx);
init_range = init_idx:(init_idx + init_window);

x0 = mean(gt_pos(init_range, 1));
y0 = mean(gt_pos(init_range, 2));

% Circular mean for yaw
sin_sum = sum(sin(gt_yaw(init_range)));
cos_sum = sum(cos(gt_yaw(init_range)));
theta0  = atan2(sin_sum, cos_sum);

%% ======================================================================
%  ONLINE GYRO BIAS ESTIMATION
%  ======================================================================
%  Use samples from init_idx onward (robot stationary at start)
bias_end_time = sensor_time(init_idx) + GYRO_BIAS_WINDOW_SEC;
bias_mask = sensor_time >= sensor_time(init_idx) & sensor_time <= bias_end_time;

if sum(bias_mask) > 10
    gyro_bias_est = mean(gyro(bias_mask, :), 1)';
else
    gyro_bias_est = zeros(3,1);
end

% --- DEBUG: Print init values (remove for submission if desired) ---
fprintf('myEKF init: idx=%d, pos=(%.3f,%.3f), yaw=%.2f deg, gyro_bias(1)=%.5f\n', ...
    init_idx, x0, y0, rad2deg(theta0), gyro_bias_est(GYRO_YAW_ROW));

%% ======================================================================
%  INITIALISE
%  ======================================================================
X = [x0; y0; 0; 0; theta0];
P = diag([0.01, 0.01, 0.1, 0.1, 0.01]);

X_Est = zeros(N, 5);
P_Est = zeros(N, 5);

for i = 1:init_idx
    X_Est(i,:) = X';
    P_Est(i,:) = diag(P)';
end

%% ======================================================================
%  PREVIOUS SENSOR VALUES
%  ======================================================================
prev_mag  = mag(init_idx, :);
prev_tof1 = tof1(init_idx, :);
prev_tof2 = tof2(init_idx, :);
prev_tof3 = tof3(init_idx, :);

gyro_hist  = zeros(ZUPT_WINDOW, 1);
accel_hist = zeros(ZUPT_WINDOW, 1);
hist_idx   = 0;
mag_step_counter = 0;

%% ======================================================================
%  MAIN EKF LOOP
%  ======================================================================
for k = (init_idx+1):N

    dt = sensor_time(k) - sensor_time(k-1);
    if dt <= 0 || dt > 0.5
        X_Est(k,:) = X';
        P_Est(k,:) = diag(P)';
        continue;
    end

    % ==================================================================
    %  PREDICTION
    % ==================================================================
    omega_z = gyro(k, GYRO_YAW_ROW) - gyro_bias_est(GYRO_YAW_ROW);

    a_body_fwd   = ACCEL_FWD_SIGN  * (accel(k, ACCEL_FWD_ROW)   - ACCEL_BIAS(ACCEL_FWD_ROW))   * ACCEL_SCALE;
    a_body_right = ACCEL_RIGHT_SIGN * (accel(k, ACCEL_RIGHT_ROW) - ACCEL_BIAS(ACCEL_RIGHT_ROW)) * ACCEL_SCALE;

    theta = X(5);
    cos_t = cos(theta);
    sin_t = sin(theta);
    a_wx = cos_t * a_body_fwd - sin_t * a_body_right;
    a_wy = sin_t * a_body_fwd + cos_t * a_body_right;

    vx_d = X(3) * VEL_DAMPING;
    vy_d = X(4) * VEL_DAMPING;

    X_pred = [X(1) + vx_d*dt + 0.5*a_wx*dt^2;
              X(2) + vy_d*dt + 0.5*a_wy*dt^2;
              vx_d + a_wx*dt;
              vy_d + a_wy*dt;
              wrapToPi(X(5) + omega_z*dt)];

    F = eye(5);
    F(1,3) = VEL_DAMPING * dt;
    F(2,4) = VEL_DAMPING * dt;
    F(3,3) = VEL_DAMPING;
    F(4,4) = VEL_DAMPING;

    Q = diag([Q_POS, Q_POS, Q_VEL, Q_VEL, Q_THETA]) * dt;
    P = F * P * F' + Q;
    X = X_pred;

    % ==================================================================
    %  ZUPT
    % ==================================================================
    gyro_norm = abs(omega_z);
    accel_total = sqrt((accel(k, ACCEL_FWD_ROW) - ACCEL_BIAS(ACCEL_FWD_ROW))^2 + ...
                       (accel(k, ACCEL_RIGHT_ROW) - ACCEL_BIAS(ACCEL_RIGHT_ROW))^2);

    hist_idx = hist_idx + 1;
    buf_pos = mod(hist_idx - 1, ZUPT_WINDOW) + 1;
    gyro_hist(buf_pos) = gyro_norm;
    accel_hist(buf_pos) = accel_total;

    if hist_idx >= ZUPT_WINDOW
        motion_metric = mean(gyro_hist) + 0.05 * mean(accel_hist);
        if motion_metric < ZUPT_THRESHOLD
            H_z = [0 0 1 0 0; 0 0 0 1 0];
            innov = -H_z * X;
            R_z = ZUPT_R * eye(2);
            S_z = H_z * P * H_z' + R_z;
            K_z = P * H_z' / S_z;
            X = X + K_z * innov;
            P = (eye(5) - K_z * H_z) * P;
            X(5) = wrapToPi(X(5));
        end
    end

    % ==================================================================
    %  MAGNETOMETER (disabled by default)
    % ==================================================================
    if MAG_ENABLE
        new_mag = any(mag(k,:) ~= prev_mag);
        if new_mag
            mag_step_counter = mag_step_counter + 1;
            prev_mag = mag(k,:);
            if mag_step_counter >= MAG_UPDATE_INTERVAL
                mag_step_counter = 0;
                mx = mag(k, 2) - MAG_HARD_IRON(2);
                my = mag(k, 3) - MAG_HARD_IRON(3);
                if (mx^2 + my^2) > 1e-12
                    mag_yaw = atan2(my, mx);
                    innov_m = wrapToPi(mag_yaw - (X(5) + YAW_OFFSET));
                    H_m = [0 0 0 0 1];
                    S_m = H_m * P * H_m' + R_MAG;
                    K_m = P * H_m' / S_m;
                    X = X + K_m * innov_m;
                    P = (eye(5) - K_m * H_m) * P;
                    X(5) = wrapToPi(X(5));
                end
            end
        end
    end

    % ==================================================================
    %  TOF UPDATE
    % ==================================================================
    tof_readings = {tof1(k,:), tof2(k,:), tof3(k,:)};
    prev_tof_all = {prev_tof1, prev_tof2, prev_tof3};

    for s = 1:3
        tof_now  = tof_readings{s};
        tof_prev = prev_tof_all{s};

        if all(tof_now == tof_prev)
            continue;
        end

        switch s
            case 1, prev_tof1 = tof_now;
            case 2, prev_tof2 = tof_now;
            case 3, prev_tof3 = tof_now;
        end

        d_meas = tof_now(1);
        if d_meas < TOF_MIN || d_meas > TOF_MAX
            continue;
        end

        ray_angle = X(5) + YAW_OFFSET + TOF_ANGLES(s);

        [d_pred, wall_id] = predictToFDistance(X(1), X(2), ray_angle, ...
            WALL_X_POS, WALL_X_NEG, WALL_Y_POS, WALL_Y_NEG);

        if isnan(d_pred) || d_pred <= 0
            continue;
        end

        innov_tof = d_meas - d_pred;

        H_tof = tofJacobian(X(1), X(2), X(5), YAW_OFFSET, TOF_ANGLES(s), ...
            wall_id, WALL_X_POS, WALL_X_NEG, WALL_Y_POS, WALL_Y_NEG);

        S_tof = H_tof * P * H_tof' + R_TOF;

        if S_tof < 1e-10
            continue;
        end

        mahal_dist = innov_tof^2 / S_tof;

        if mahal_dist > MAHAL_GATE
            continue;
        end

        K_tof = P * H_tof' / S_tof;
        X = X + K_tof * innov_tof;
        P = (eye(5) - K_tof * H_tof) * P;
        X(5) = wrapToPi(X(5));
    end

    % ==================================================================
    %  ARENA BOUNDS
    % ==================================================================
    if ARENA_BOUND_ENABLE
        bmax_x = WALL_X_POS - ARENA_MARGIN;
        bmin_x = WALL_X_NEG + ARENA_MARGIN;
        bmax_y = WALL_Y_POS - ARENA_MARGIN;
        bmin_y = WALL_Y_NEG + ARENA_MARGIN;

        if X(1) > bmax_x
            [X, P] = boundUpdate(X, P, 1, bmax_x, R_BOUND);
        elseif X(1) < bmin_x
            [X, P] = boundUpdate(X, P, 1, bmin_x, R_BOUND);
        end
        if X(2) > bmax_y
            [X, P] = boundUpdate(X, P, 2, bmax_y, R_BOUND);
        elseif X(2) < bmin_y
            [X, P] = boundUpdate(X, P, 2, bmin_y, R_BOUND);
        end
    end

    % ==================================================================
    %  STORE
    % ==================================================================
    X_Est(k,:) = X';
    P_Est(k,:) = diag(P)';
end

end  % END myEKF


%% ======================================================================
%  HELPER FUNCTIONS
%  ======================================================================

function [gyro, accel, mag, tof1, tof2, tof3, sensor_time, gt_pos, gt_rot] = extractData(out)
    sensor_time = out.Sensor_Time.time(:);
    N = length(sensor_time);

    gyro = squeeze(out.Sensor_GYRO.signals.values);
    if size(gyro,1)==3 && size(gyro,2)==N, gyro=gyro'; end

    accel = squeeze(out.Sensor_ACCEL.signals.values);
    if size(accel,1)==3 && size(accel,2)==N, accel=accel'; end

    mag = squeeze(out.Sensor_MAG.signals.values);
    if size(mag,1)==3 && size(mag,2)==N, mag=mag'; end

    tof1 = squeeze(out.Sensor_ToF1.signals.values);
    if size(tof1,1)==4 && size(tof1,2)==N, tof1=tof1'; end
    tof2 = squeeze(out.Sensor_ToF2.signals.values);
    if size(tof2,1)==4 && size(tof2,2)==N, tof2=tof2'; end
    tof3 = squeeze(out.Sensor_ToF3.signals.values);
    if size(tof3,1)==4 && size(tof3,2)==N, tof3=tof3'; end

    gt_pos = squeeze(out.GT_position.signals.values);
    if size(gt_pos,1)==3 && size(gt_pos,2)==N, gt_pos=gt_pos'; end

    gt_rot = squeeze(out.GT_rotation.signals.values);
    if size(gt_rot,1)==4 && size(gt_rot,2)==N, gt_rot=gt_rot'; end
end


function [d, wall_id] = predictToFDistance(px, py, ray_angle, xp, xn, yp, yn)
    cos_a = cos(ray_angle);
    sin_a = sin(ray_angle);
    d = Inf; wall_id = 0;

    if cos_a > 1e-6
        t = (xp - px) / cos_a;
        hy = py + t * sin_a;
        if t > 0 && hy >= yn && hy <= yp && t < d, d = t; wall_id = 1; end
    end
    if cos_a < -1e-6
        t = (xn - px) / cos_a;
        hy = py + t * sin_a;
        if t > 0 && hy >= yn && hy <= yp && t < d, d = t; wall_id = 2; end
    end
    if sin_a > 1e-6
        t = (yp - py) / sin_a;
        hx = px + t * cos_a;
        if t > 0 && hx >= xn && hx <= xp && t < d, d = t; wall_id = 3; end
    end
    if sin_a < -1e-6
        t = (yn - py) / sin_a;
        hx = px + t * cos_a;
        if t > 0 && hx >= xn && hx <= xp && t < d, d = t; wall_id = 4; end
    end

    if isinf(d), d = NaN; wall_id = 0; end
end


function H = tofJacobian(px, py, theta, yaw_offset, tof_angle, wall_id, xp, xn, yp, yn)
    ray = theta + yaw_offset + tof_angle;
    cos_a = cos(ray);
    sin_a = sin(ray);

    switch wall_id
        case 1
            dd_dpx = -1/cos_a; dd_dpy = 0;
            dd_dray = (xp - px) * sin_a / cos_a^2;
        case 2
            dd_dpx = -1/cos_a; dd_dpy = 0;
            dd_dray = (xn - px) * sin_a / cos_a^2;
        case 3
            dd_dpx = 0; dd_dpy = -1/sin_a;
            dd_dray = -(yp - py) * cos_a / sin_a^2;
        case 4
            dd_dpx = 0; dd_dpy = -1/sin_a;
            dd_dray = -(yn - py) * cos_a / sin_a^2;
        otherwise
            H = zeros(1,5); return;
    end

    H = [dd_dpx, dd_dpy, 0, 0, dd_dray];
end


function [X, P] = boundUpdate(X, P, state_idx, bound_val, R_b)
    H = zeros(1, 5);
    H(state_idx) = 1;
    innov = bound_val - X(state_idx);
    S = H * P * H' + R_b;
    K = P * H' / S;
    X = X + K * innov;
    P = (eye(5) - K * H) * P;
end


function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end