function [X_Est, P_Est, GT] = myEKF(out)
    % Extended Kalman Filter — Unified Offline & Real-Time
    %
    % OFFLINE (coursework grading):
    %   Called ONCE with full logged dataset struct.
    %   Returns X_Est [Nx5], P_Est [5x5xN], GT [Nx3]
    %
    % REAL-TIME (robot deployment):
    %   Called REPEATEDLY, each time with a single-timestep struct
    %   containing only the latest sensor readings.
    %   Returns X_Est [1x5], P_Est [5x5], GT []
    %   Persistent variables maintain filter state between calls.
    %
    %   To RESET the filter (e.g. at robot startup), call:
    %       myEKF(struct('reset', true, 'GT_position', ..., 'Sensor_MAG', ...))

    %% Persistent Filter State
    persistent X P prev_time is_initialised
    persistent gyro_bias accel_bias mag_hard_iron mag_soft_iron mag_global_offset
    persistent calib_gyro_buf calib_accel_buf calib_count calib_done
    persistent last_imu_idx last_mag_idx last_tof1_idx last_tof2_idx last_tof3_idx

    %% Constants
    CALIB_SAMPLES = 50;
    arena_bounds = struct('x_max', 2.0, 'x_min', -2.0, 'y_max', 2.0, 'y_min', -2.0);
    alpha_tof = [0, pi/2, -pi/2];   % Forward, Left, Right mount angles

    % Noise parameters
    Q = diag([0.001, 0.001, 0.01, 0.5, 0.5]);
    R_mag = 0.1;
    R_tof = 0.05;
    gamma_thresh = 9.0;  % Mahalanobis gate (3 sigma)

    % Magnetometer calibration constants (from offline calibration data)
    MAG_HARD_IRON = [5.7e-06, -3.7e-05];
    MAG_SOFT_IRON = [3.7500, 0.5769];

    %% Data Extraction
    imu_time = out.Sensor_GYRO.time;
    gyro_data = squeeze(out.Sensor_GYRO.signals.values)';
    accel_data = squeeze(out.Sensor_ACCEL.signals.values)';

    if size(gyro_data, 2) > size(gyro_data, 1)
        gyro_data = gyro_data';
    end

    if size(accel_data, 2) > size(accel_data, 1)
        accel_data = accel_data';
    end

    mag_time = out.Sensor_MAG.time;
    mag_data = squeeze(out.Sensor_MAG.signals.values)';

    if size(mag_data, 2) > size(mag_data, 1)
        mag_data = mag_data';
    end

    [tof1_time, tof1_data] = extract_tof(out.Sensor_ToF1);
    [tof2_time, tof2_data] = extract_tof(out.Sensor_ToF2);
    [tof3_time, tof3_data] = extract_tof(out.Sensor_ToF3);

    gt_pos = squeeze(out.GT_position.signals.values);

    if size(gt_pos, 1) == 3
        gt_pos = gt_pos';
    end

    GT = gt_pos;

    num_steps = length(imu_time);

    %% Mode Detection
    % Offline: called once with full dataset (num_steps > 1)
    % Real-time: called repeatedly with a single new step (num_steps == 1)
    offline_mode = (num_steps > 1);

    %% Filter Initialisation
    % Always re-initialise in offline mode
    % In real-time, only initialise if persistent state is empty or reset requested
    needs_init = offline_mode || isempty(is_initialised) || ~is_initialised || (isfield(out, 'reset') && out.reset);

    if needs_init
        mag_hard_iron = MAG_HARD_IRON;
        mag_soft_iron = MAG_SOFT_IRON;

        % Initial position from first ground truth point
        x0 = gt_pos(1, 1);
        y0 = gt_pos(1, 2);

        % Initial yaw from first magnetometer reading
        mag_x0 = (mag_data(1, 1) - mag_hard_iron(1)) * mag_soft_iron(1);
        mag_y0 = (mag_data(1, 2) - mag_hard_iron(2)) * mag_soft_iron(2);
        theta0 = atan2(mag_y0, mag_x0);

        % Compute mag to arena alignment offset using first 10 GT points
        if size(gt_pos, 1) >= 10
            dy = gt_pos(10, 2) - gt_pos(1, 2);
            dx = gt_pos(10, 1) - gt_pos(1, 1);

            if abs(dx) > 1e-4 || abs(dy) > 1e-4
                theta_gt = atan2(dy, dx);
                mag_global_offset = wrapToPi(theta_gt - theta0);
            else
                mag_global_offset = 0;
            end

        else
            mag_global_offset = 0;
        end

        % Initial state vector X
        X = [x0; y0; theta0; 0; 0];

        % Initial covariance matrix P
        P = diag([0.1, 0.1, 0.5, 1.0, 1.0]);

        % Calibration buffers (biases estimated over first CALIB_SAMPLES steps)
        gyro_bias = zeros(1, 3);
        accel_bias = zeros(1, 3);
        calib_gyro_buf = zeros(CALIB_SAMPLES, 3);
        calib_accel_buf = zeros(CALIB_SAMPLES, 3);
        calib_count = 0;
        calib_done = false;

        prev_time = imu_time(1);
        is_initialised = true;

        % Index pointers for tracking asynchronous sensor updates
        last_imu_idx = 0;
        last_mag_idx = 0;
        last_tof1_idx = 0;
        last_tof2_idx = 0;
        last_tof3_idx = 0;
    end

    %% Step Range
    % Offline: process every step in the dataset
    % Real-time: process only new steps since last call
    if offline_mode
        start_k = 1;
        end_k = num_steps;
        mag_start = 1;
        tof1_start = 1;
        tof2_start = 1;
        tof3_start = 1;
    else
        start_k = last_imu_idx + 1;
        end_k = num_steps;
        mag_start = last_mag_idx + 1;
        tof1_start = last_tof1_idx + 1;
        tof2_start = last_tof2_idx + 1;
        tof3_start = last_tof3_idx + 1;
    end

    % Output arrays
    n_out = end_k - start_k + 1;
    X_Est = zeros(n_out, 5);
    P_Est = zeros(5, 5, n_out);

    % Indices to keep track of asynchronous sensor updates
    mag_idx = mag_start;
    tof1_idx = tof1_start;
    tof2_idx = tof2_start;
    tof3_idx = tof3_start;

    %% Main EKF Loop
    for k = start_k:end_k
        out_idx = k - start_k + 1;
        current_time = imu_time(k);

        % Time step
        dt = current_time - prev_time;

        if dt <= 0
            dt = 1e-3;
        end

        prev_time = current_time;

        % Calibration window — accumulate IMU samples before trusting biases
        if ~calib_done
            calib_count = calib_count + 1;
            calib_gyro_buf(calib_count, :) = gyro_data(k, :);
            calib_accel_buf(calib_count, :) = accel_data(k, :);

            if calib_count >= CALIB_SAMPLES
                gyro_bias = mean(calib_gyro_buf, 1);
                accel_bias = mean(calib_accel_buf, 1);
                calib_done = true;
            end

            X_Est(out_idx, :) = X';
            P_Est(:, :, out_idx) = P;

            continue
        end

        % Calibrate IMU inputs
        omega = gyro_data(k, 3) - gyro_bias(3);
        ax = accel_data(k, 1) - accel_bias(1);
        ay = accel_data(k, 2) - accel_bias(2);

        % Predict Step
        X_pred = zeros(5, 1);
        X_pred(1) = X(1) + (X(4)*cos(X(3)) - X(5)*sin(X(3))) * dt;
        X_pred(2) = X(2) + (X(4)*sin(X(3)) + X(5)*cos(X(3))) * dt;
        X_pred(3) = wrapToPi(X(3) + omega * dt);
        X_pred(4) = X(4) + ax * dt;
        X_pred(5) = X(5) + ay * dt;

        % Calculate State Jacobian (F)
        F = eye(5);
        F(1,3) = (-X(4)*sin(X(3)) - X(5)*cos(X(3))) * dt;
        F(1,4) = cos(X(3)) * dt;
        F(1,5) = -sin(X(3)) * dt;
        F(2,3) = (X(4)*cos(X(3)) - X(5)*sin(X(3))) * dt;
        F(2,4) = sin(X(3)) * dt;
        F(2,5) = cos(X(3)) * dt;

        % Predict Covariance
        X = X_pred;
        P = F * P * F' + Q;

        % Update Step
        % Magnetometer Update
        while mag_idx <= length(mag_time) && current_time >= mag_time(mag_idx)
            mag_x_cal = (mag_data(mag_idx, 1) - mag_hard_iron(1)) * mag_soft_iron(1);
            mag_y_cal = (mag_data(mag_idx, 2) - mag_hard_iron(2)) * mag_soft_iron(2);

            z_mag = wrapToPi(atan2(mag_y_cal, mag_x_cal) + mag_global_offset);
            H_mag = [0 0 1 0 0];
            S_mag = H_mag * P * H_mag' + R_mag;
            innovation = wrapToPi(z_mag - X(3));
            K_mag = P * H_mag' / S_mag;

            X = X + K_mag * innovation;
            X(3) = wrapToPi(X(3));
            P = (eye(5) - K_mag * H_mag) * P;

            mag_idx = mag_idx + 1;
        end

        % ToF 1 Update (Forward)
        while tof1_idx <= length(tof1_time) && current_time >= tof1_time(tof1_idx)
            if tof1_data(tof1_idx, 4) == 0
                [X, P] = tof_update(X, P, tof1_data(tof1_idx, 1), arena_bounds, alpha_tof(1), R_tof, gamma_thresh);
            end

            tof1_idx = tof1_idx + 1;
        end

        % ToF 2 Update (Left)
        while tof2_idx <= length(tof2_time) && current_time >= tof2_time(tof2_idx)
            if tof2_data(tof2_idx, 4) == 0
                [X, P] = tof_update(X, P, tof2_data(tof2_idx, 1), arena_bounds, alpha_tof(2), R_tof, gamma_thresh);
            end

            tof2_idx = tof2_idx + 1;
        end

        % ToF 3 Update (Right)
        while tof3_idx <= length(tof3_time) && current_time >= tof3_time(tof3_idx)
            if tof3_data(tof3_idx, 4) == 0
                [X, P] = tof_update(X, P, tof3_data(tof3_idx, 1), arena_bounds, alpha_tof(3), R_tof, gamma_thresh);
            end

            tof3_idx = tof3_idx + 1;
        end

        X_Est(out_idx, :) = X';
        P_Est(:, :, out_idx) = P;
    end

    % Update persistent index pointers for next real-time call
    if ~offline_mode
        last_imu_idx = end_k;
        last_mag_idx = mag_idx - 1;
        last_tof1_idx = tof1_idx - 1;
        last_tof2_idx = tof2_idx - 1;
        last_tof3_idx = tof3_idx - 1;
    end
end


%% Helper Functions

function [X_out, P_out] = tof_update(X, P, raw_dist, bounds, alpha, R_tof, gamma)
    % Performs a single ToF EKF update step with Mahalanobis gating
    [h_x, H_tof] = calculate_expected_tof(X, bounds, alpha);
    S = H_tof * P * H_tof' + R_tof;
    innovation = raw_dist - h_x;

    if (innovation^2) / S > gamma
        X_out = X;
        P_out = P;
        return
    end

    K = P * H_tof' / S;
    X_out = X + K * innovation;
    X_out(3) = wrapToPi(X_out(3));
    P_out = (eye(5) - K * H_tof) * P;
end

function [h_x, H_tof] = calculate_expected_tof(X, bounds, alpha)
    % Calculates the expected ToF distance and the Jacobian H matrix
    x = X(1); y = X(2); theta = X(3);
    phi = wrapToPi(theta + alpha);  % Global angle of the sensor beam

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

    % Find the wall the sensor beam hits first (minimum positive distance)
    [h_x, wall_idx] = min([d_right, d_left, d_top, d_bottom]);

    % Calculate Jacobian depending on which wall was hit
    H_tof = zeros(1, 5);

    if wall_idx <= 2
        % Hit a vertical wall (right or left)
        H_tof(1) = -1 / cos(phi);
        H_tof(3) = h_x * tan(phi);
    else
        % Hit a horizontal wall (top or bottom)
        H_tof(2) = -1 / sin(phi);
        H_tof(3) = -h_x * cot(phi);
    end
end

function [t, d] = extract_tof(sensor)
    % Safely extracts time and data arrays from a ToF sensor struct
    t = sensor.time;
    d = squeeze(sensor.signals.values);
    
    if size(d, 1) == 4
        d = d';
    end
end

function angle_wrapped = wrapToPi(angle)
    % Wraps an angle to the [-pi, pi] range
    angle_wrapped = mod(angle + pi, 2*pi) - pi;
end