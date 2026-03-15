function [X_Est, P_Est, GT] = myEKF(out)
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

    gt_time = out.GT_time.signals.values;
    gt_time = gt_time - gt_time(1);
    gt_pos = squeeze(out.GT_position.signals.values);
    if size(gt_pos, 1) == 3, gt_pos = gt_pos'; end
    GT = gt_pos;

    num_steps = length(imu_time);

    %% Calibration Constants
    % --- PASTE YOUR VALUES FROM CALIBRATION.M HERE ---
    gyro_bias_x = 0.00186;  % rad/s (Confirm this with your script)
    accel_bias_x = 9.84855;     % <--- PASTE ACCEL X BIAS HERE
    accel_bias_y = 0.07485;     % <--- PASTE ACCEL Y BIAS HERE
 
    % Magnetometer: horizontal axes are Y (axis 2) and Z (axis 3)
    mag_hard_iron = [-3.705e-05,  4.415e-05];  % [Y, Z] hard iron offsets (T)
    mag_soft_iron = [1.0958,      0.9196];      % [Y, Z] soft iron scale factors

    % Arena bounds 
    arena_bounds = struct('x_max', 1.2, 'x_min', -1.2, 'y_max', 1.2, 'y_min', -2.16);

    % ToF mounting angles relative to robot forward axis
    alpha_tof1 = 0;       % Forward
    alpha_tof2 = pi/2;    % Left
    alpha_tof3 = -pi/2;   % Right

    %% Filter Initialization
    x0 = gt_pos(1, 1);
    y0 = gt_pos(1, 2);

    % Initial heading from GT quaternion
    gt_rot = squeeze(out.GT_rotation.signals.values);
    if size(gt_rot, 1) == 4, gt_rot = gt_rot'; end
    qw = gt_rot(1,1); qx = gt_rot(1,2); qy = gt_rot(1,3); qz = gt_rot(1,4);
    theta0 = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));

    % Magnetometer global offset
    mag_y0 = (mag_data(1, 2) - mag_hard_iron(1)) * mag_soft_iron(1);
    mag_z0 = (mag_data(1, 3) - mag_hard_iron(2)) * mag_soft_iron(2);
    z_mag0 = atan2(mag_z0, mag_y0);
    mag_global_offset = wrapToPi(theta0 - z_mag0);

    % State Vector: [x; y; theta; v_body_x; v_body_y]
    X = [x0; y0; theta0; 0; 0];
    P = diag([0.1, 0.1, 0.1, 2.0, 2.0]);

    % Process Noise Matrix (Q)
    % Tuned down pos/theta since they are deterministically integrated.
    % Velocity variance is higher to trust the accelerometer data.
    Q = diag([1e-4, 1e-4, 1e-4, 0.1, 0.1]);

    R_mag = 0.1;
    R_tof = 0.01;

    % Outlier Rejection Threshold (Reduced to block 'holes in the wall')
    gamma_threshold = 3.84;

    X_Est = zeros(num_steps, 5);
    P_Est = zeros(5,5,num_steps);

    prev_time = imu_time(1);
    mag_idx = 1; tof1_idx = 1; tof2_idx = 1; tof3_idx = 1;

    %% Main EKF Loop
    for k = 1:num_steps
        current_time = imu_time(k);
        dt = current_time - prev_time;
        if dt <= 0, dt = 0.001; end
        prev_time = current_time;

        % --- PREDICTION STEP (Now using real IMU data!) ---
        omega = -(gyro_data(k, 1) - gyro_bias_x); 
        ax = accel_data(k, 1) - accel_bias_x; 
        ay = accel_data(k, 2) - accel_bias_y;

        X_pred = zeros(5,1);
        % Position update using body-frame velocities
        X_pred(1) = X(1) + (X(4)*cos(X(3)) - X(5)*sin(X(3))) * dt;
        X_pred(2) = X(2) + (X(4)*sin(X(3)) + X(5)*cos(X(3))) * dt;
        % Heading update
        X_pred(3) = wrapToPi(X(3) + omega * dt);
        % Velocity update using body-frame accelerations
        X_pred(4) = X(4) + ax * dt;
        X_pred(5) = X(5) + ay * dt;

        % Jacobian of state transition matrix (F)
        F = eye(5);
        F(1,3) = (-X(4)*sin(X(3)) - X(5)*cos(X(3))) * dt;
        F(1,4) = cos(X(3)) * dt;
        F(1,5) = -sin(X(3)) * dt;
        F(2,3) = (X(4)*cos(X(3)) - X(5)*sin(X(3))) * dt;
        F(2,4) = sin(X(3)) * dt;
        F(2,5) = cos(X(3)) * dt;

        P_pred = F * P * F' + Q;

        X = X_pred;
        P = P_pred;

        % --- MEASUREMENT UPDATES ---

        % Magnetometer Update
        if mag_idx <= length(mag_time) && current_time >= mag_time(mag_idx)
            mag_y_cal = (mag_data(mag_idx, 2) - mag_hard_iron(1)) * mag_soft_iron(1);
            mag_z_cal = (mag_data(mag_idx, 3) - mag_hard_iron(2)) * mag_soft_iron(2);
            z_mag = wrapToPi(atan2(mag_z_cal, mag_y_cal) + mag_global_offset);

            H_mag = [0 0 1 0 0];
            S_mag = H_mag * P * H_mag' + R_mag;
            innovation_mag = wrapToPi(z_mag - X(3));

            K_mag = P * H_mag' / S_mag;
            X = X + K_mag * innovation_mag;
            X(3) = wrapToPi(X(3));
            IKH = eye(5) - K_mag * H_mag;
            P = IKH * P * IKH' + K_mag * R_mag * K_mag'; 
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
                    IKH = eye(5) - K * H_tof;
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
                    IKH = eye(5) - K * H_tof;
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
                    IKH = eye(5) - K * H_tof;
                    P = IKH * P * IKH' + K * R_tof * K'; 
                end
            end
            tof3_idx = tof3_idx + 1;
        end

        X_Est(k, :) = X';
        P_Est(:, :, k) = P;
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
    [h_x, wall_idx] = min(distances);
    H_tof = zeros(1, 5);

    if wall_idx == 1 || wall_idx == 2
        H_tof(1) = -1 / cos(phi);
        H_tof(3) = h_x * tan(phi);
    else
        H_tof(2) = -1 / sin(phi);
        H_tof(3) = -h_x * cot(phi);
    end
end

function angle_wrapped = wrapToPi(angle)
    angle_wrapped = mod(angle + pi, 2*pi) - pi;
end