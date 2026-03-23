function [X_Est, P_Est] = myEKF(out)
% Extended Kalman Filter — Sub-Terranean Navigation
% State: X = [x; y; theta; vx_world; vy_world; b_gyro]
% Architecture: world-frame velocities, mag anomaly detection,
%               ToF rate heading, ZUPT, RTS smoother

%% SECTION 1: Data Extraction
    imu_time   = out.Sensor_GYRO.time;
    gyro_data  = squeeze(out.Sensor_GYRO.signals.values)';
    accel_data = squeeze(out.Sensor_ACCEL.signals.values)';

    mag_time = out.Sensor_MAG.time;
    mag_data = squeeze(out.Sensor_MAG.signals.values)';

    tof1_time = out.Sensor_ToF1.time;
    tof1_data = squeeze(out.Sensor_ToF1.signals.values);
    if size(tof1_data, 1) == 4, tof1_data = tof1_data'; end

    tof2_time = out.Sensor_ToF2.time;
    tof2_data = squeeze(out.Sensor_ToF2.signals.values);
    if size(tof2_data, 1) == 4, tof2_data = tof2_data'; end

    tof3_time = out.Sensor_ToF3.time;
    tof3_data = squeeze(out.Sensor_ToF3.signals.values);
    if size(tof3_data, 1) == 4, tof3_data = tof3_data'; end

    gt_pos = squeeze(out.GT_position.signals.values);
    if size(gt_pos, 1) == 3, gt_pos = gt_pos'; end

    num_steps = length(imu_time);
    dt        = 0.005;

%% SECTION 2: Per-Run Calibration
    % Hardcoded fallback constants
    gyro_bias      =  0.00186;
    accel_bias_fwd = -0.396;
    accel_bias_lat =  0.07485;

    % Find first run of 100+ consecutive samples with gyro norm < 0.01 rad/s
    search_limit = min(500, num_steps);
    stat_start   = 0;
    consec_count = 0;
    for i = 1:search_limit
        if abs(gyro_data(i, 1)) < 0.01
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
        n_stat     = min(200, num_steps - stat_start + 1);
        stat_range = stat_start : (stat_start + n_stat - 1);
        gyro_bias      = mean(gyro_data(stat_range, 1));
        accel_bias_fwd = mean(accel_data(stat_range, 3));
        accel_bias_lat = mean(accel_data(stat_range, 2));
    end

%% SECTION 3: Arena Wall Estimation
    theta0 = pi/2;
    x0     = gt_pos(1, 1);
    y0     = gt_pos(1, 2);

    % Default hardcoded bounds
    arena_bounds = struct('x_max',  1.22, 'x_min', -1.22, ...
                          'y_max',  1.22, 'y_min', -1.22);

    if stat_start > 0
        % At theta0 = pi/2: fwd_dy=1, rgt_dx=1, lft_dx=-1
        fwd_dy = sin(theta0);    % 1
        rgt_dx = sin(theta0);    % 1
        lft_dx = -sin(theta0);   % -1

        % Genuine ToF readings in stat_range (value changed from previous)
        tof1_in  = tof1_data(stat_range, 1);
        tof2_in  = tof2_data(stat_range, 1);
        tof3_in  = tof3_data(stat_range, 1);

        genuine1 = tof1_in([true; diff(tof1_in) ~= 0]);
        genuine2 = tof2_in([true; diff(tof2_in) ~= 0]);
        genuine3 = tof3_in([true; diff(tof3_in) ~= 0]);

        genuine1 = genuine1(genuine1 > 0);
        genuine2 = genuine2(genuine2 > 0);
        genuine3 = genuine3(genuine3 > 0);

        if ~isempty(genuine1) && ~isempty(genuine2) && ~isempty(genuine3)
            d_rgt = median(genuine1);   % ToF1 faces right
            d_fwd = median(genuine2);   % ToF2 faces forward
            d_lft = median(genuine3);   % ToF3 faces left

            if ~isnan(d_rgt) && ~isnan(d_fwd) && ~isnan(d_lft) && ...
               d_rgt > 0    &&  d_fwd > 0    &&  d_lft > 0
                wall_y_max = y0 + d_fwd * fwd_dy;   % y0 + d_fwd
                wall_x_max = x0 + d_rgt * rgt_dx;   % x0 + d_rgt
                wall_x_min = x0 + d_lft * lft_dx;   % x0 - d_lft
                wall_y_min = -wall_y_max;             % symmetry

                arena_bounds = struct('x_max', wall_x_max, 'x_min', wall_x_min, ...
                                      'y_max', wall_y_max, 'y_min', wall_y_min);
            end
        end
    end

%% SECTION 4: Initialisation
    % Magnetometer calibration constants (locked)
    mag_hard_iron = [-3.705e-05,  4.415e-05];
    mag_soft_iron = [ 1.0958,     0.9196   ];

    % Magnetometer global offset (theta0 = pi/2 hardcoded)
    mag_y_cal0        = (mag_data(1, 2) - mag_hard_iron(1)) * mag_soft_iron(1);
    mag_z_cal0        = (mag_data(1, 3) - mag_hard_iron(2)) * mag_soft_iron(2);
    z_mag0            = atan2(mag_z_cal0, mag_y_cal0);
    mag_global_offset = wrapToPi(theta0 - z_mag0);

    % ToF mounting angles (locked)
    alpha_tof1 = -pi/2;
    alpha_tof2 =  0;
    alpha_tof3 =  pi/2;

    % Initial state: world-frame velocities = 0, bias seeded from calibration
    X = [x0; y0; theta0; 0; 0; gyro_bias];

    % Covariance
    P = diag([0.1, 0.1, 0.1, 2.0, 2.0, 0.25]);

    % Process noise (Q values scaled by dt per step)
    Q = diag([1e-4, 1e-4, 1.1e-3, 0.5, 0.5, 1.5e-5]) * dt;

    % Measurement noise
    R_mag           = 4.0;
    R_tof           = 0.01;
    gamma_threshold = 9.0;

    % Magnetometer anomaly detection
    theta_gyro_ref          = theta0;
    gyro_ref_bias           = gyro_bias;
    mag_disable_until       = 0;
    mag_innov_buf           = zeros(20, 1);
    mag_innov_count         = 0;
    gyro_mag_diverge_count  = 0;
    prev_innovation_mag     = 0;
    mag_innov_growing_count = 0;

    % ToF rate tracking
    prev_tof1_dist         = NaN;
    prev_tof1_genuine_time = NaN;
    prev_tof2_dist         = NaN;
    prev_tof2_genuine_time = NaN;
    prev_tof3_dist         = NaN;
    prev_tof3_genuine_time = NaN;

    % RTS smoother storage
    X_fwd = zeros(num_steps, 6);
    P_fwd = zeros(6, 6, num_steps);
    X_prd = zeros(num_steps, 6);
    P_prd = zeros(6, 6, num_steps);
    F_all = zeros(6, 6, num_steps);

    % Sensor index trackers
    mag_idx  = 1;
    tof1_idx = 1;
    tof2_idx = 1;
    tof3_idx = 1;

%% SECTION 5: Main EKF Loop
    for k = 1:num_steps
        current_time = imu_time(k);

        % ---- 5a: Prediction (world-frame velocities) ----
        omega = gyro_data(k, 1) - X(6);

        ax_body = max(-2.0, min(2.0, accel_data(k, 3) - accel_bias_fwd));
        ay_body = max(-2.0, min(2.0, accel_data(k, 2) - accel_bias_lat));

        % Rotate body-frame acceleration to world frame
        ax_world = ax_body * cos(X(3)) - ay_body * sin(X(3));
        ay_world = ax_body * sin(X(3)) + ay_body * cos(X(3));

        vel_damp = 1.0 - 0.5 * dt;

        X_pred    = zeros(6, 1);
        X_pred(1) = X(1) + X(4) * dt;
        X_pred(2) = X(2) + X(5) * dt;
        X_pred(3) = wrapToPi(X(3) + omega * dt);
        X_pred(4) = X(4) * vel_damp + ax_world * dt;
        X_pred(5) = X(5) * vel_damp + ay_world * dt;
        X_pred(6) = X(6);

        % Jacobian F (world-frame, 6x6)
        F       = eye(6);
        F(1, 4) = dt;
        F(2, 5) = dt;
        F(3, 6) = -dt;
        F(4, 3) = (-ax_body * sin(X(3)) - ay_body * cos(X(3))) * dt;
        F(5, 3) = ( ax_body * cos(X(3)) - ay_body * sin(X(3))) * dt;
        F(4, 4) = vel_damp;
        F(5, 5) = vel_damp;

        % Adaptive process noise
        Q_k = Q;
        if abs(omega) > 0.15
            Q_k(3, 3) = Q_k(3, 3) * (1.0 + 9.0 * (abs(omega) - 0.15));
        end
        if k <= mag_disable_until
            Q_k(4, 4) = Q_k(4, 4) * 1.5;
            Q_k(5, 5) = Q_k(5, 5) * 1.5;
        end
        speed = sqrt(X(4)^2 + X(5)^2);
        if speed > 0.05
            Q_k(6, 6) = Q_k(6, 6) * (1.0 + 5.0 * speed);
        end

        P_pred = F * P * F' + Q_k;

        % Store for RTS (prediction stored before measurement updates)
        X_prd(k, :)    = X_pred';
        P_prd(:, :, k) = P_pred;
        F_all(:, :, k) = F;

        % Accept prediction
        X = X_pred;
        P = P_pred;

        % ---- 5b: Independent gyro reference ----
        theta_gyro_ref = wrapToPi(theta_gyro_ref + ...
                         (gyro_data(k, 1) - gyro_ref_bias) * dt);

        % ---- 5c: Magnetometer update with anomaly detection ----
        if mag_idx <= length(mag_time) && current_time >= mag_time(mag_idx)
            mag_y_cal = (mag_data(mag_idx, 2) - mag_hard_iron(1)) * mag_soft_iron(1);
            mag_z_cal = (mag_data(mag_idx, 3) - mag_hard_iron(2)) * mag_soft_iron(2);
            z_mag     = wrapToPi(atan2(mag_z_cal, mag_y_cal) + mag_global_offset);

            innovation_mag = wrapToPi(z_mag - X(3));

            % Criterion 1: Innovation buffer (variance + RMS thresholds)
            mag_innov_count         = mag_innov_count + 1;
            buf_idx                 = mod(mag_innov_count - 1, 20) + 1;
            mag_innov_buf(buf_idx)  = innovation_mag;
            if mag_innov_count >= 20
                innov_var = var(mag_innov_buf);
                innov_rms = sqrt(mean(mag_innov_buf.^2));
                if innov_var > 0.04 || innov_rms > 0.15
                    mag_disable_until = k + 400;
                end
            end

            % Criterion 2: Innovation magnitude growing for 5 consecutive steps
            if abs(innovation_mag) > abs(prev_innovation_mag) + 0.02
                mag_innov_growing_count = mag_innov_growing_count + 1;
                if mag_innov_growing_count >= 5
                    mag_disable_until       = k + 400;
                    mag_innov_growing_count = 0;
                end
            else
                mag_innov_growing_count = max(0, mag_innov_growing_count - 1);
            end
            prev_innovation_mag = innovation_mag;

            % Criterion 3: Sustained absolute divergence from gyro reference
            gyro_mag_diff = abs(wrapToPi(z_mag - theta_gyro_ref));
            elapsed       = current_time;
            max_expected  = 0.008 * elapsed + 0.12;
            if gyro_mag_diff > max_expected
                gyro_mag_diverge_count = gyro_mag_diverge_count + 1;
                if gyro_mag_diverge_count >= 50
                    mag_disable_until      = k + 400;
                    gyro_mag_diverge_count = 0;
                end
            else
                gyro_mag_diverge_count = max(0, gyro_mag_diverge_count - 1);
            end

            % Apply update only when magnetometer not disabled
            if k > mag_disable_until
                H_mag = [0 0 1 0 0 0];
                S_mag = H_mag * P * H_mag' + R_mag;
                if innovation_mag^2 / S_mag < gamma_threshold
                    K_mag = P * H_mag' / S_mag;
                    X     = X + K_mag * innovation_mag;
                    X(3)  = wrapToPi(X(3));
                    IKH   = eye(6) - K_mag * H_mag;
                    P     = IKH * P * IKH' + K_mag * R_mag * K_mag';
                    % Slowly track gyro ref bias toward EKF estimate
                    gyro_ref_bias = 0.95 * gyro_ref_bias + 0.05 * X(6);
                end
            end

            mag_idx = mag_idx + 1;
        end

        % ---- 5d/5e: ToF distance + rate updates ----

        % --- ToF 1 (right-facing, alpha = -pi/2) ---
        if tof1_idx <= length(tof1_time) && current_time >= tof1_time(tof1_idx)
            if tof1_data(tof1_idx, 4) == 0
                raw_dist1    = tof1_data(tof1_idx, 1);
                [h_x, H_tof] = calculate_expected_tof(X, arena_bounds, alpha_tof1);
                S            = H_tof * P * H_tof' + R_tof;
                innovation   = raw_dist1 - h_x;
                if (innovation^2) / S < gamma_threshold
                    K    = P * H_tof' / S;
                    X    = X + K * innovation;
                    X(3) = wrapToPi(X(3));
                    IKH  = eye(6) - K * H_tof;
                    P    = IKH * P * IKH' + K * R_tof * K';
                end

                % Genuine reading detection
                tof1_is_new = isnan(prev_tof1_dist) || (raw_dist1 ~= prev_tof1_dist);

                % Rate-based heading update (mag disabled only)
                if tof1_is_new && ~isnan(prev_tof1_dist) && k <= mag_disable_until
                    dt_genuine = current_time - prev_tof1_genuine_time;
                    if dt_genuine > 0.08
                        speed_now = sqrt(X(4)^2 + X(5)^2);
                        if speed_now > 0.05 && abs(omega) < 0.2
                            phi1       = wrapToPi(X(3) + alpha_tof1);
                            d_rate_obs = (raw_dist1 - prev_tof1_dist) / dt_genuine;
                            d_rate_exp = -(X(4) * cos(phi1) + X(5) * sin(phi1));
                            H_rate     = zeros(1, 6);
                            H_rate(3)  = X(4) * sin(phi1) - X(5) * cos(phi1);
                            H_rate(4)  = -cos(phi1);
                            H_rate(5)  = -sin(phi1);
                            S_rate     = H_rate * P * H_rate' + 0.5;
                            innov_rate = d_rate_obs - d_rate_exp;
                            if innov_rate^2 < 9.0 * S_rate
                                K_rate = P * H_rate' / S_rate;
                                X      = X + K_rate * innov_rate;
                                X(3)   = wrapToPi(X(3));
                                P      = (eye(6) - K_rate * H_rate) * P;
                            end
                        end
                    end
                end
                if tof1_is_new
                    prev_tof1_dist         = raw_dist1;
                    prev_tof1_genuine_time = current_time;
                end
            end
            tof1_idx = tof1_idx + 1;
        end

        % --- ToF 2 (forward-facing, alpha = 0) ---
        if tof2_idx <= length(tof2_time) && current_time >= tof2_time(tof2_idx)
            if tof2_data(tof2_idx, 4) == 0
                raw_dist2    = tof2_data(tof2_idx, 1);
                [h_x, H_tof] = calculate_expected_tof(X, arena_bounds, alpha_tof2);
                S            = H_tof * P * H_tof' + R_tof;
                innovation   = raw_dist2 - h_x;
                if (innovation^2) / S < gamma_threshold
                    K    = P * H_tof' / S;
                    X    = X + K * innovation;
                    X(3) = wrapToPi(X(3));
                    IKH  = eye(6) - K * H_tof;
                    P    = IKH * P * IKH' + K * R_tof * K';
                end

                tof2_is_new = isnan(prev_tof2_dist) || (raw_dist2 ~= prev_tof2_dist);

                if tof2_is_new && ~isnan(prev_tof2_dist) && k <= mag_disable_until
                    dt_genuine = current_time - prev_tof2_genuine_time;
                    if dt_genuine > 0.08
                        speed_now = sqrt(X(4)^2 + X(5)^2);
                        if speed_now > 0.05 && abs(omega) < 0.2
                            phi2       = wrapToPi(X(3) + alpha_tof2);
                            d_rate_obs = (raw_dist2 - prev_tof2_dist) / dt_genuine;
                            d_rate_exp = -(X(4) * cos(phi2) + X(5) * sin(phi2));
                            H_rate     = zeros(1, 6);
                            H_rate(3)  = X(4) * sin(phi2) - X(5) * cos(phi2);
                            H_rate(4)  = -cos(phi2);
                            H_rate(5)  = -sin(phi2);
                            S_rate     = H_rate * P * H_rate' + 0.5;
                            innov_rate = d_rate_obs - d_rate_exp;
                            if innov_rate^2 < 9.0 * S_rate
                                K_rate = P * H_rate' / S_rate;
                                X      = X + K_rate * innov_rate;
                                X(3)   = wrapToPi(X(3));
                                P      = (eye(6) - K_rate * H_rate) * P;
                            end
                        end
                    end
                end
                if tof2_is_new
                    prev_tof2_dist         = raw_dist2;
                    prev_tof2_genuine_time = current_time;
                end
            end
            tof2_idx = tof2_idx + 1;
        end

        % --- ToF 3 (left-facing, alpha = +pi/2) ---
        if tof3_idx <= length(tof3_time) && current_time >= tof3_time(tof3_idx)
            if tof3_data(tof3_idx, 4) == 0
                raw_dist3    = tof3_data(tof3_idx, 1);
                [h_x, H_tof] = calculate_expected_tof(X, arena_bounds, alpha_tof3);
                S            = H_tof * P * H_tof' + R_tof;
                innovation   = raw_dist3 - h_x;
                if (innovation^2) / S < gamma_threshold
                    K    = P * H_tof' / S;
                    X    = X + K * innovation;
                    X(3) = wrapToPi(X(3));
                    IKH  = eye(6) - K * H_tof;
                    P    = IKH * P * IKH' + K * R_tof * K';
                end

                tof3_is_new = isnan(prev_tof3_dist) || (raw_dist3 ~= prev_tof3_dist);

                if tof3_is_new && ~isnan(prev_tof3_dist) && k <= mag_disable_until
                    dt_genuine = current_time - prev_tof3_genuine_time;
                    if dt_genuine > 0.08
                        speed_now = sqrt(X(4)^2 + X(5)^2);
                        if speed_now > 0.05 && abs(omega) < 0.2
                            phi3       = wrapToPi(X(3) + alpha_tof3);
                            d_rate_obs = (raw_dist3 - prev_tof3_dist) / dt_genuine;
                            d_rate_exp = -(X(4) * cos(phi3) + X(5) * sin(phi3));
                            H_rate     = zeros(1, 6);
                            H_rate(3)  = X(4) * sin(phi3) - X(5) * cos(phi3);
                            H_rate(4)  = -cos(phi3);
                            H_rate(5)  = -sin(phi3);
                            S_rate     = H_rate * P * H_rate' + 0.5;
                            innov_rate = d_rate_obs - d_rate_exp;
                            if innov_rate^2 < 9.0 * S_rate
                                K_rate = P * H_rate' / S_rate;
                                X      = X + K_rate * innov_rate;
                                X(3)   = wrapToPi(X(3));
                                P      = (eye(6) - K_rate * H_rate) * P;
                            end
                        end
                    end
                end
                if tof3_is_new
                    prev_tof3_dist         = raw_dist3;
                    prev_tof3_genuine_time = current_time;
                end
            end
            tof3_idx = tof3_idx + 1;
        end

        % ---- 5f: ZUPT (three-part) ----
        if k >= 40
            accel_win_fwd = accel_data(k-39:k, 3) - accel_bias_fwd;
            accel_win_lat = accel_data(k-39:k, 2) - accel_bias_lat;
            gyro_win      = gyro_data(k-39:k, 1);

            az_var   = var(accel_win_fwd);
            ay_var   = var(accel_win_lat);
            gyro_var = var(gyro_win);

            if az_var < 0.005 && ay_var < 0.005 && gyro_var < 0.0005
                % Part 1: Velocity ZUPT
                H_v     = [0 0 0 1 0 0; 0 0 0 0 1 0];
                innov_v = -[X(4); X(5)];
                S_v     = H_v * P * H_v' + 0.002 * eye(2);
                K_v     = P * H_v' / S_v;
                X       = X + K_v * innov_v;
                P       = (eye(6) - K_v * H_v) * P;

                % Part 2: Gyro bias ZUPT (stationary => gyro reading = bias)
                H_b     = [0 0 0 0 0 1];
                innov_b = gyro_data(k, 1) - X(6);
                S_b     = P(6, 6) + 0.00005;
                K_b     = P * H_b' / S_b;
                X       = X + K_b * innov_b;
                P       = (eye(6) - K_b * H_b) * P;
                gyro_ref_bias = X(6);

                % Part 3: Soft heading ZUPT (no rotation when stationary)
                H_h     = [0 0 1 0 0 0];
                innov_h = -(omega * dt);
                S_h     = H_h * P * H_h' + 0.005;
                K_h     = P * H_h' / S_h;
                X       = X + K_h * innov_h;
                X(3)    = wrapToPi(X(3));
                P       = (eye(6) - K_h * H_h) * P;
            end
        end

        % Store filtered estimates for RTS smoother
        X_fwd(k, :)    = X';
        P_fwd(:, :, k) = P;

    end  % for k = 1:num_steps

%% SECTION 6: RTS Smoother (backward pass)
    X_s = X_fwd;
    P_s = P_fwd;

    for k = num_steps-1 : -1 : 1
        P_prd_reg    = P_prd(:, :, k+1) + 1e-9 * eye(6);
        G            = P_fwd(:, :, k) * F_all(:, :, k+1)' / P_prd_reg;
        G(3, :)      = 0;   % Block heading row (safe first pass)
        X_s(k, :)    = X_fwd(k, :) + (G * (X_s(k+1, :)' - X_prd(k+1, :)'))';
        P_s(:, :, k) = P_fwd(:, :, k) + G * (P_s(:, :, k+1) - P_prd(:, :, k+1)) * G';
    end

%% SECTION 7: Output Conversion (from smoothed estimates)
    X_Est = X_s(:, 1:5);
    P_Est = P_s(1:5, 1:5, :);

end  % function myEKF

%% Helper Functions

function [h_x, H_tof] = calculate_expected_tof(X, bounds, alpha)
    x   = X(1);
    y   = X(2);
    phi = wrapToPi(X(3) + alpha);

    d_right  = inf;
    d_left   = inf;
    d_top    = inf;
    d_bottom = inf;

    if cos(phi) > 1e-6
        d_right = (bounds.x_max - x) / cos(phi);
    elseif cos(phi) < -1e-6
        d_left  = (bounds.x_min - x) / cos(phi);
    end

    if sin(phi) > 1e-6
        d_top    = (bounds.y_max - y) / sin(phi);
    elseif sin(phi) < -1e-6
        d_bottom = (bounds.y_min - y) / sin(phi);
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
        H_tof(3) = 0;
    else
        H_tof(2) = -1 / sin(phi);
        H_tof(3) = 0;
    end
end

function angle_wrapped = wrapToPi(angle)
    angle_wrapped = mod(angle + pi, 2*pi) - pi;
end
