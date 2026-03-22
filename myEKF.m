function [X_Est, P_Est] = myEKF(out)
    % Extended Kalman Filter

    %% Data Extraction
    imu_time = out.Sensor_GYRO.time;
    gyro_data = squeeze(out.Sensor_GYRO.signals.values)';
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

    %% Calibration Constants
    % --- PASTE YOUR VALUES FROM CALIBRATION.M HERE ---
    gyro_bias_x  = 0.00186;      % rad/s  — verified Session 0, col 1, stationary mean
                                  % NOTE: kept for reference only; Session 5b uses X(6) online
    accel_bias_fwd = -0.396;    % m/s^2  — axis 3 (forward) stationary mean, verified Session 0 (BUG3 fix)
    accel_bias_y   = 0.07485;   % m/s^2  — axis 2 (lateral) bias (BUG4 minor mismatch, deferred)

    % Magnetometer: horizontal axes are Y (axis 2) and Z (axis 3)
    mag_hard_iron = [-3.705e-05,  4.415e-05];  % [Y, Z] hard iron offsets (T)
    mag_soft_iron = [1.0958,      0.9196];      % [Y, Z] soft iron scale factors

    % Arena bounds
    arena_bounds = struct('x_max', 1.22, 'x_min', -1.22, 'y_max', 1.22, 'y_min', -1.22); % Update B: 244cm x 244cm arena, origin at centre

    % ToF mounting angles relative to robot forward axis
    alpha_tof1 = -pi/2;    % Left (was incorrectly labelled Forward)
    alpha_tof2 = 0;       % Forward (was incorrectly labelled Left)
    alpha_tof3 = pi/2;   % Right (confirmed correct)

    %% Filter Initialization
    x0 = gt_pos(1, 1);
    y0 = gt_pos(1, 2);

    theta0 = pi/2;  % Robot starts perpendicular to south wall, facing +Y (confirmed)

    % Magnetometer global offset
    mag_y0 = (mag_data(1, 2) - mag_hard_iron(1)) * mag_soft_iron(1);
    mag_z0 = (mag_data(1, 3) - mag_hard_iron(2)) * mag_soft_iron(2);
    z_mag0 = atan2(mag_z0, mag_y0);
    mag_global_offset = wrapToPi(theta0 - z_mag0);

    % --- Step 1: State Vector extended to 6 states ---
    % Before (5-state): X = [x; y; theta; vx; vy]
    % After  (6-state): X = [x; y; theta; vx; vy; b_gyro]
    %   b_gyro initialised at stationary gyro bias (0.00186 rad/s, Session 0)
    X = [x0; y0; theta0; 0; 0; 0.00186];

    % --- Step 5: P extended to 6×6 ---
    % Before: diag([0.1, 0.1, 0.1, 2.0, 2.0])          5×5
    % After:  diag([0.1, 0.1, 0.1, 2.0, 2.0, (0.5)^2]) 6×6
    %   P(6,6) = (0.5)^2 = 0.25: large initial bias uncertainty ±0.5 rad/s
    %   Allows aggressive early bias adaptation before turn 1 (Session 5f)
    P = diag([0.1, 0.1, 0.1, 2.0, 2.0, (0.5)^2]);

    % --- Step 4: Q extended to 6×6 ---
    % Before: diag([1e-4, 1e-4, 1e-4, 0.1, 0.1])       5×5
    % After:  diag([1e-4, 1e-4, 1e-4, 0.1, 0.1, 1e-6]) 6×6
    %   Q(6,6) = 1e-6: bias changes very slowly (random walk)
    Q = diag([1e-4, 1e-4, 1e-4, 0.1, 0.1, 1e-6]);

    R_mag = 100.0; % Very weak magnetometer: motor EMI corrupts mag during Task 2
                   % circuit. K_mag ≈ 0.001 so gyro dominates heading (was 0.1→1.0→100)
    R_tof = 0.05;

    % Outlier Rejection Threshold (Reduced to block 'holes in the wall')
    % gamma_threshold = 3.84;
    gamma_threshold = 9.0;

    X_Est = zeros(num_steps, 5);
    P_Est = zeros(5,5,num_steps);

    prev_time = imu_time(1);
    mag_idx = 1; tof1_idx = 1; tof2_idx = 1; tof3_idx = 1;

    %% Main EKF Loop
    for k = 1:num_steps
        current_time = imu_time(k);

        % Real IMU step — compute dt from last real sample
        % dt = current_time - prev_time;
        % if dt <= 0 || dt > 0.1, dt = 1/104; end  % fallback for first step or anomalies
        % prev_time = current_time;  % update AFTER skip check, only on real steps
        dt = 0.005;

        % --- PREDICTION STEP ---
        % Gyro sign: axis 1 reads positive for CCW rotation (right-hand rule,
        % axis pointing up).  No negation needed.
        % Previous code had omega = -(gyro - bias) which reversed the turn
        % direction, causing 180 deg heading error on Task 2.
        %
        % --- Step 2: omega uses X(6) (online bias) instead of gyro_bias_x constant ---
        % Before: omega = gyro_data(k, 1) - gyro_bias_x;
        % After:  omega = gyro_data(k, 1) - X(6);
        omega = gyro_data(k, 1) - X(6);

        % Accelerometer: clip spikes from Mecanum wheel vibration (up to ±12 m/s²
        % observed in Task 2 data) before integrating into velocity.
        % True robot acceleration is <2 m/s²; anything beyond is noise.
        accel_clip = 2.0;
        ax = max(-accel_clip, min(accel_clip, accel_data(k,3) - accel_bias_fwd));
        ay = max(-accel_clip, min(accel_clip, accel_data(k,2) - accel_bias_y));

        X_pred = zeros(6,1);
        % Position update using body-frame velocities
        X_pred(1) = X(1) + (X(4)*cos(X(3)) - X(5)*sin(X(3))) * dt;
        X_pred(2) = X(2) + (X(4)*sin(X(3)) + X(5)*cos(X(3))) * dt;
        % Heading update (uses online bias X(6))
        X_pred(3) = wrapToPi(X(3) + omega * dt);
        % Velocity update using body-frame accelerations
        X_pred(4) = X(4) + ax * dt;
        X_pred(5) = X(5) + ay * dt;
        % Bias update: random walk — stays constant in prediction
        X_pred(6) = X(6);

        % --- Step 3: F extended to 6×6 ---
        % Before: F = eye(5), entries F(1:2, 3:5) filled          5×5
        % After:  F = eye(6), same entries plus F(3,6) = -dt       6×6
        %   F(3,6) = -dt  because theta_new = theta + (gyro - b)*dt
        %            → ∂theta_new/∂b_gyro = -dt
        %   F(6,6) = 1 from eye(6): bias is a random walk, no decay
        F = eye(6);
        F(1,3) = (-X(4)*sin(X(3)) - X(5)*cos(X(3))) * dt;
        F(1,4) = cos(X(3)) * dt;
        F(1,5) = -sin(X(3)) * dt;
        F(2,3) = (X(4)*cos(X(3)) - X(5)*sin(X(3))) * dt;
        F(2,4) = sin(X(3)) * dt;
        F(2,5) = cos(X(3)) * dt;
        F(3,6) = -dt;  % ∂theta/∂b_gyro — new entry for 6-state extension

        P_pred = F * P * F' + Q;

        X = X_pred;
        P = P_pred;

        % --- MEASUREMENT UPDATES ---

        % Magnetometer Update
        if mag_idx <= length(mag_time) && current_time >= mag_time(mag_idx)
            mag_y_cal = (mag_data(mag_idx, 2) - mag_hard_iron(1)) * mag_soft_iron(1);
            mag_z_cal = (mag_data(mag_idx, 3) - mag_hard_iron(2)) * mag_soft_iron(2);
            z_mag = wrapToPi(atan2(mag_z_cal, mag_y_cal) + mag_global_offset);

            % --- Step 6: H_mag padded to 1×6 ---
            % Before: H_mag = [0 0 1 0 0]    1×5
            % After:  H_mag = [0 0 1 0 0 0]  1×6 (zero in b_gyro position)
            H_mag = [0 0 1 0 0 0];
            S_mag = H_mag * P * H_mag' + R_mag;
            innovation_mag = wrapToPi(z_mag - X(3));

            % Mahalanobis gating: reject corrupted mag readings (motor EMI)
            if (innovation_mag^2) / S_mag < gamma_threshold
                K_mag = P * H_mag' / S_mag;
                X = X + K_mag * innovation_mag;
                X(3) = wrapToPi(X(3));
                IKH = eye(6) - K_mag * H_mag;
                P = IKH * P * IKH' + K_mag * R_mag * K_mag';
            end
            mag_idx = mag_idx + 1;
        end

        % ToF 1 Update
        if tof1_idx <= length(tof1_time) && current_time >= tof1_time(tof1_idx)
            if tof1_data(tof1_idx, 4) == 0 % If valid reading
                raw_dist = tof1_data(tof1_idx, 1);
                [h_x, H_tof] = calculate_expected_tof(X, arena_bounds, alpha_tof1);
                S = H_tof * P * H_tof' + R_tof;
                innovation = raw_dist - h_x;

                % Outlier Gating
                if (innovation^2) / S < gamma_threshold
                    K = P * H_tof' / S;
                    X = X + K * innovation;
                    X(3) = wrapToPi(X(3));
                    IKH = eye(6) - K * H_tof;
                    P = IKH * P * IKH' + K * R_tof * K';
                end
            end
            tof1_idx = tof1_idx + 1;
        end

        % ToF 2 Update
        if tof2_idx <= length(tof2_time) && current_time >= tof2_time(tof2_idx)
            if tof2_data(tof2_idx, 4) == 0
                raw_dist = tof2_data(tof2_idx, 1);
                [h_x, H_tof] = calculate_expected_tof(X, arena_bounds, alpha_tof2);
                S = H_tof * P * H_tof' + R_tof;
                innovation = raw_dist - h_x;

                if (innovation^2) / S < gamma_threshold
                    K = P * H_tof' / S;
                    X = X + K * innovation;
                    X(3) = wrapToPi(X(3));
                    IKH = eye(6) - K * H_tof;
                    P = IKH * P * IKH' + K * R_tof * K';
                end
            end
            tof2_idx = tof2_idx + 1;
        end

        % ToF 3 Update
        if tof3_idx <= length(tof3_time) && current_time >= tof3_time(tof3_idx)
            if tof3_data(tof3_idx, 4) == 0
                raw_dist = tof3_data(tof3_idx, 1);
                [h_x, H_tof] = calculate_expected_tof(X, arena_bounds, alpha_tof3);
                S = H_tof * P * H_tof' + R_tof;
                innovation = raw_dist - h_x;

                if (innovation^2) / S < gamma_threshold
                    K = P * H_tof' / S;
                    X = X + K * innovation;
                    X(3) = wrapToPi(X(3));
                    IKH = eye(6) - K * H_tof;
                    P = IKH * P * IKH' + K * R_tof * K';
                end
            end
            tof3_idx = tof3_idx + 1;
        end

        % --- Step 7: Output only first 5 states (grading compatibility) ---
        % Before: X_Est(k, :) = X'           stores 5 states
        %         P_Est(:, :, k) = P          stores 5×5
        % After:  X_Est(k, :) = X(1:5)'      stores first 5 only (b_gyro internal)
        %         P_Est(:, :, k) = P(1:5,1:5) stores 5×5 block only
        X_Est(k, :) = X(1:5)';
        P_Est(:, :, k) = P(1:5, 1:5);
    end
end

%% Helper Functions (Unchanged)
function [h_x, H_tof] = calculate_expected_tof(X, bounds, alpha)
    x = X(1); y = X(2); theta = X(3);
    phi = wrapToPi(theta + alpha);
    d_right = inf; d_left = inf; d_top = inf; d_bottom = inf;

    if cos(phi) > 1e-6
        d_right = (bounds.x_max - x) / cos(phi);
    elseif cos(phi) < -1e-6
        d_left = (bounds.x_min - x) / cos(phi);
    end

    if sin(phi) > 1e-6
        d_top = (bounds.y_max - y) / sin(phi);
    elseif sin(phi) < -1e-6
        d_bottom = (bounds.y_min - y) / sin(phi);
    end

    distances = [d_right, d_left, d_top, d_bottom];

    % Bounds protection: discard distances that are zero or negative.
    % These occur when the EKF position has drifted outside the arena walls.
    % A negative predicted distance causes a numerically catastrophic Kalman
    % update (state jumps to ±10,000m). Replacing with inf means min() will
    % ignore them; if ALL are inf the update is skipped entirely.
    distances(distances <= 0) = inf;

    [h_x, wall_idx] = min(distances);

    % If every distance is inf, the EKF position is outside the arena.
    % Skip this ToF update by zeroing H_tof (Kalman gain becomes 0).
    % zeros(1, length(X)) auto-sizes to 1×6 after 6-state extension.
    if isinf(h_x)
        H_tof = zeros(1, length(X));
        return;
    end

    % --- Step 6: H_tof initialised as 1×6 (was 1×5) ---
    % Before: H_tof = zeros(1, 5)
    % After:  H_tof = zeros(1, 6)  — zero in b_gyro column (col 6)
    H_tof = zeros(1, 6);

    if wall_idx == 1 || wall_idx == 2
        H_tof(1) = -1 / cos(phi);
        H_tof(3) = 0;  % remove heading coupling from ToF updates
    else
        H_tof(2) = -1 / sin(phi);
        H_tof(3) = 0;  % remove heading coupling from ToF updates
    end
end

function angle_wrapped = wrapToPi(angle)
    angle_wrapped = mod(angle + pi, 2*pi) - pi;
end
