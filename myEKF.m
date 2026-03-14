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
    if size(tof1_data, 1) == 4
        tof1_data = tof1_data';
    end

    tof2_time = out.Sensor_ToF2.time;
    tof2_data = squeeze(out.Sensor_ToF2.signals.values);
    if size(tof2_data, 1) == 4
        tof2_data = tof2_data';
    end

    tof3_time = out.Sensor_ToF3.time;
    tof3_data = squeeze(out.Sensor_ToF3.signals.values);
    if size(tof3_data, 1) == 4
        tof3_data = tof3_data';
    end

    gt_time = out.GT_time.signals.values;
    gt_time = gt_time - gt_time(1);
    gt_pos = squeeze(out.GT_position.signals.values);
    if size(gt_pos, 1) == 3
        gt_pos = gt_pos';
    end
    GT = gt_pos;

    num_steps = length(imu_time);

    %% Calibration Constants
    % Offline gyro bias (axis 1 = yaw axis, from calibration.m stationary data)
    gyro_bias_x = 0.00186;  % rad/s

    % Magnetometer: horizontal axes are Y (axis 2) and Z (axis 3)
    % Values from calibration.m (calib1_rotate.mat)
    mag_hard_iron = [-3.705e-05,  4.415e-05];  % [Y, Z] hard iron offsets (T)
    mag_soft_iron = [1.0958,      0.9196];      % [Y, Z] soft iron scale factors

    % Arena bounds (computed from ToF readings at known start position + GT)
    arena_bounds = struct('x_max', 1.2, 'x_min', -1.2, 'y_max', 1.2, 'y_min', -2.16);

    % ToF mounting angles relative to robot forward axis
    alpha_tof1 = 0;       % Forward  (+X when theta=0)
    alpha_tof2 = pi/2;    % Left     (+Y when theta=0)
    alpha_tof3 = -pi/2;   % Right    (-Y when theta=0)

    %% Pre-compute ToF velocity measurements
    % Derivation: d(ToF)/dt = -(vx*cos(alpha) + vy*sin(alpha))  [body frame]
    % So z_vel = -d(ToF)/dt = vx*cos(alpha) + vy*sin(alpha)
    % H_vel = [0, 0, 0, cos(alpha), sin(alpha)]
    % This gives DIRECT body-frame velocity observability — the missing link
    % that causes the filter to underestimate velocity by ~50% otherwise.
    tof1_vel = zeros(length(tof1_time), 1);
    tof1_vel_valid = false(length(tof1_time), 1);
    for i = 2:length(tof1_time)
        if tof1_data(i,4) == 0 && tof1_data(i-1,4) == 0
            dt_v = tof1_time(i) - tof1_time(i-1);
            if dt_v > 1e-6
                tof1_vel(i) = -(tof1_data(i,1) - tof1_data(i-1,1)) / dt_v;
                tof1_vel_valid(i) = true;
            end
        end
    end

    tof2_vel = zeros(length(tof2_time), 1);
    tof2_vel_valid = false(length(tof2_time), 1);
    for i = 2:length(tof2_time)
        if tof2_data(i,4) == 0 && tof2_data(i-1,4) == 0
            dt_v = tof2_time(i) - tof2_time(i-1);
            if dt_v > 1e-6
                tof2_vel(i) = -(tof2_data(i,1) - tof2_data(i-1,1)) / dt_v;
                tof2_vel_valid(i) = true;
            end
        end
    end

    tof3_vel = zeros(length(tof3_time), 1);
    tof3_vel_valid = false(length(tof3_time), 1);
    for i = 2:length(tof3_time)
        if tof3_data(i,4) == 0 && tof3_data(i-1,4) == 0
            dt_v = tof3_time(i) - tof3_time(i-1);
            if dt_v > 1e-6
                tof3_vel(i) = -(tof3_data(i,1) - tof3_data(i-1,1)) / dt_v;
                tof3_vel_valid(i) = true;
            end
        end
    end

    %% Filter Initialization
    x0 = gt_pos(1, 1);
    y0 = gt_pos(1, 2);

    % Initial heading from GT quaternion [W, X, Y, Z]
    gt_rot = squeeze(out.GT_rotation.signals.values);
    if size(gt_rot, 1) == 4, gt_rot = gt_rot'; end
    qw = gt_rot(1,1); qx = gt_rot(1,2); qy = gt_rot(1,3); qz = gt_rot(1,4);
    theta0 = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));

    % Magnetometer global offset
    mag_y0 = (mag_data(1, 2) - mag_hard_iron(1)) * mag_soft_iron(1);
    mag_z0 = (mag_data(1, 3) - mag_hard_iron(2)) * mag_soft_iron(2);
    z_mag0 = atan2(mag_z0, mag_y0);
    mag_global_offset = wrapToPi(theta0 - z_mag0);

    X = [x0; y0; theta0; 0; 0];
    P = diag([0.1, 0.1, 0.1, 2.0, 2.0]);

    Q = diag([0.005, 0.005, 0.001, 1.0, 1.0]);

    R_mag = 0.1;
    R_tof = 0.01;

    % Velocity measurement noise: Var[z_vel] = 2*sigma_tof^2 / dt_tof^2
    % sigma_tof = 0.01 m (conservative), dt_tof ~ 0.1 s → R_vel = 0.02
    % Using 0.05 for robustness against irregular ToF intervals.
    R_vel = 0.05;

    gamma_threshold = 100;

    X_Est = zeros(num_steps, 5);
    P_Est = zeros(5,5,num_steps);

    prev_time = imu_time(1);

    mag_idx = 1;
    tof1_idx = 1;
    tof2_idx = 1;
    tof3_idx = 1;

    %% Main EKF Loop
    for k = 1:num_steps
        current_time = imu_time(k);
        dt = current_time - prev_time;

        if dt <= 0
            dt = 0.001;
        end
        prev_time = current_time;

        omega = -(gyro_data(k, 1) - gyro_bias_x);
        ax = 0;
        ay = 0;

        X_pred = zeros(5,1);
        X_pred(1) = X(1) + (X(4)*cos(X(3)) - X(5)*sin(X(3))) * dt;
        X_pred(2) = X(2) + (X(4)*sin(X(3)) + X(5)*cos(X(3))) * dt;
        X_pred(3) = X(3) + omega * dt;
        X_pred(4) = X(4) + ax * dt;
        X_pred(5) = X(5) + ay * dt;

        X_pred(3) = wrapToPi(X_pred(3));

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

        % --- Magnetometer update ---
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
            P = IKH * P * IKH' + K_mag * R_mag * K_mag';  % Joseph form

            mag_idx = mag_idx + 1;
        end

        % --- ToF1 position + velocity update ---
        if tof1_idx <= length(tof1_time) && current_time >= tof1_time(tof1_idx)
            raw_dist = tof1_data(tof1_idx, 1);
            if tof1_data(tof1_idx, 4) == 0
                [h_x, H_tof] = calculate_expected_tof(X, arena_bounds, alpha_tof1);
                S = H_tof * P * H_tof' + R_tof;
                innovation = raw_dist - h_x;
                if (innovation^2) / S < gamma_threshold
                    K = P * H_tof' / S;
                    X = X + K * innovation;
                    X(3) = wrapToPi(X(3));
                    IKH = eye(5) - K * H_tof;
                    P = IKH * P * IKH' + K * R_tof * K';  % Joseph form
                end
                % Velocity update: z_vel = -d(ToF1)/dt = vx*cos(0)+vy*sin(0) = vx
                if tof1_vel_valid(tof1_idx)
                    H_vel = [0, 0, 0, cos(alpha_tof1), sin(alpha_tof1)];
                    h_vel = X(4)*cos(alpha_tof1) + X(5)*sin(alpha_tof1);
                    S_vel = H_vel * P * H_vel' + R_vel;
                    innov_vel = tof1_vel(tof1_idx) - h_vel;
                    if innov_vel^2 / S_vel < gamma_threshold
                        K_vel = P * H_vel' / S_vel;
                        X = X + K_vel * innov_vel;
                        X(3) = wrapToPi(X(3));
                        IKH_v = eye(5) - K_vel * H_vel;
                        P = IKH_v * P * IKH_v' + K_vel * R_vel * K_vel';
                    end
                end
            end
            tof1_idx = tof1_idx + 1;
        end

        % --- ToF2 position + velocity update ---
        if tof2_idx <= length(tof2_time) && current_time >= tof2_time(tof2_idx)
            raw_dist = tof2_data(tof2_idx, 1);
            if tof2_data(tof2_idx, 4) == 0
                [h_x, H_tof] = calculate_expected_tof(X, arena_bounds, alpha_tof2);
                S = H_tof * P * H_tof' + R_tof;
                innovation = raw_dist - h_x;
                if (innovation^2) / S < gamma_threshold
                    K = P * H_tof' / S;
                    X = X + K * innovation;
                    X(3) = wrapToPi(X(3));
                    IKH = eye(5) - K * H_tof;
                    P = IKH * P * IKH' + K * R_tof * K';  % Joseph form
                end
                % Velocity update: z_vel = -d(ToF2)/dt = vx*cos(pi/2)+vy*sin(pi/2) = vy
                if tof2_vel_valid(tof2_idx)
                    H_vel = [0, 0, 0, cos(alpha_tof2), sin(alpha_tof2)];
                    h_vel = X(4)*cos(alpha_tof2) + X(5)*sin(alpha_tof2);
                    S_vel = H_vel * P * H_vel' + R_vel;
                    innov_vel = tof2_vel(tof2_idx) - h_vel;
                    if innov_vel^2 / S_vel < gamma_threshold
                        K_vel = P * H_vel' / S_vel;
                        X = X + K_vel * innov_vel;
                        X(3) = wrapToPi(X(3));
                        IKH_v = eye(5) - K_vel * H_vel;
                        P = IKH_v * P * IKH_v' + K_vel * R_vel * K_vel';
                    end
                end
            end
            tof2_idx = tof2_idx + 1;
        end

        % --- ToF3 position + velocity update ---
        if tof3_idx <= length(tof3_time) && current_time >= tof3_time(tof3_idx)
            raw_dist = tof3_data(tof3_idx, 1);
            if tof3_data(tof3_idx, 4) == 0
                [h_x, H_tof] = calculate_expected_tof(X, arena_bounds, alpha_tof3);
                S = H_tof * P * H_tof' + R_tof;
                innovation = raw_dist - h_x;
                if (innovation^2) / S < gamma_threshold
                    K = P * H_tof' / S;
                    X = X + K * innovation;
                    X(3) = wrapToPi(X(3));
                    IKH = eye(5) - K * H_tof;
                    P = IKH * P * IKH' + K * R_tof * K';  % Joseph form
                end
                % Velocity update: z_vel = -d(ToF3)/dt = vx*cos(-pi/2)+vy*sin(-pi/2) = -vy
                if tof3_vel_valid(tof3_idx)
                    H_vel = [0, 0, 0, cos(alpha_tof3), sin(alpha_tof3)];
                    h_vel = X(4)*cos(alpha_tof3) + X(5)*sin(alpha_tof3);
                    S_vel = H_vel * P * H_vel' + R_vel;
                    innov_vel = tof3_vel(tof3_idx) - h_vel;
                    if innov_vel^2 / S_vel < gamma_threshold
                        K_vel = P * H_vel' / S_vel;
                        X = X + K_vel * innov_vel;
                        X(3) = wrapToPi(X(3));
                        IKH_v = eye(5) - K_vel * H_vel;
                        P = IKH_v * P * IKH_v' + K_vel * R_vel * K_vel';
                    end
                end
            end
            tof3_idx = tof3_idx + 1;
        end

        X_Est(k, :) = X';
        P_Est(:, :, k) = P;
    end
end

%% Helper Functions
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
