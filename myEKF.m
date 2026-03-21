function [X_Est, P_Est] = myEKF(out)
% myEKF - Extended Kalman Filter for 2D robot localisation inside a walled arena.
%
%   [X_Est, P_Est] = myEKF(out)
%
%   Inputs:
%       out   - Struct of logged sensor data (IMU, magnetometer, ToF, ground truth)
%   Outputs:
%       X_Est - [N x 5] state estimates [x, y, theta, vx, vy] at each IMU timestep
%       P_Est - [5 x 5 x N] state covariance matrices at each IMU timestep
%
%   State vector: [x (m); y (m); theta (rad); vx (m/s); vy (m/s)]
%   Sensors fused: Gyroscope (prediction), Magnetometer (heading), 3x ToF (position)
%
%   BOARD ORIENTATION (confirmed by calib1_rotate.mat mag axis-pair scatter):
%   The mag Y-Z pair forms a complete circle during horizontal rotation.
%   Board X is therefore the vertical axis (world +Z).
%     Board X --> World +Z  (vertical, up)
%     Board Y --> horizontal plane axis 1
%     Board Z --> horizontal plane axis 2
%   Gyro yaw rate = gyro(:,1)  [Board X = world +Z, CCW positive]
%   Mag heading   = atan2(mag(:,3), mag(:,2))  [Board Z, Board Y]

    %% =========================================================================
    %  1. DATA EXTRACTION
    % ==========================================================================
    imu_time  = out.Sensor_GYRO.time;
    gyro_data = squeeze(out.Sensor_GYRO.signals.values)';  % [N x 3] rad/s

    mag_time  = out.Sensor_MAG.time;
    mag_data  = squeeze(out.Sensor_MAG.signals.values)';   % [M x 3] Tesla

    tof_times = {out.Sensor_ToF1.time, out.Sensor_ToF2.time, out.Sensor_ToF3.time};
    tof_raw   = {squeeze(out.Sensor_ToF1.signals.values), ...
                 squeeze(out.Sensor_ToF2.signals.values), ...
                 squeeze(out.Sensor_ToF3.signals.values)};

    for i = 1:3
        if size(tof_raw{i}, 1) == 4, tof_raw{i} = tof_raw{i}'; end
    end

    % Ground truth used ONLY for initial pose — never fused into the filter
    gt_pos = squeeze(out.GT_position.signals.values);
    if size(gt_pos, 1) == 3, gt_pos = gt_pos'; end

    %% =========================================================================
    %  2. CALIBRATION CONSTANTS
    %     All values derived offline by calibrate_sensors.m.
    %     To update: re-run calibrate_sensors.m and paste the output block here.
    % ==========================================================================

    % ----- Gyroscope (calib2_straight.mat stationary window) -----------------
    gyro_bias  = [0.001928, -0.011091, -0.001405]; % [X, Y, Z] rad/s — X is yaw axis
    gyro_scale = 1.0061;                            % dimensionless (calib1_rotate.mat)

    % ----- Magnetometer (calib1_rotate.mat, horizontal axes Y=2 and Z=3) -----
    % NOTE: ellipticity = 0.36 — above the 0.05 target. Heading correction will
    % be imperfect; R_mag is kept loose (0.5) to limit the magnetometer's influence.
    mag_hard = [-3.7050e-05,  4.4150e-05]; % [offset_Y, offset_Z] Tesla
    mag_soft = [1.0958, 0.9196];           % [scale_Y,  scale_Z]  dimensionless

    % ----- ToF noise (calib2_straight.mat stationary variance) ---------------
    R_tof = 0.000075;  % mean variance across all three sensors [m^2]
                       % individual std: ToF1=0.00821m  ToF2=0.00885m  ToF3=0.00882m

    % ----- EKF tuning ---------------------------------------------------------
    % R_mag: very loose trust on magnetometer.
    % Calibration ellipticity = 0.36 (target < 0.05) — sensor is unreliable.
    % With R_mag = 0.5, K_mag ≈ 0.02 per update; at 50 Hz the bad heading
    % reading pulls 1.6 rad of error into X(3) within ~1 second.
    % At R_mag = 10.0, K_mag ≈ 0.001 — gyro dominates, mag provides only a
    % very gentle long-term drift correction (~0.08 rad/s influence).
    % Gyro bias drift over 15s = 0.001928 × 15 = 0.029 rad — far more reliable.
    R_mag = 0.5;

    % Q: process noise — low on position/heading (gyro-driven), high on velocity
    Q = diag([1e-4, 1e-4, 1e-7, 0.5, 0.5]);

    % Innovation gates for ToF: chi2(0.99,1) statistical + hard absolute cap
    % Rejects wall-hole false readings and large outliers
    gate_chi2 = 6.63;   % chi-squared threshold
    gate_abs  = 0.45;   % absolute cap [m]

    % Velocity decay: first-order fade between 10 Hz ToF corrections
    % tau ~ 0.3s: exp(-dt/tau) ~ exp(-0.01/0.3) at ~100 Hz IMU rate
    vel_decay = 0.975;

    % ----- Arena geometry -----------------------------------------------------
    bounds    = struct('x_max', 1.2, 'x_min', -1.2, 'y_max', 1.65, 'y_min', -1.65);
    alpha_tofs = [0, -pi/2, pi/2];  % ToF1=forward, ToF2=left, ToF3=right [rad]

    %% =========================================================================
    %  3. FILTER INITIALISATION
    % ==========================================================================
    gt_rot = squeeze(out.GT_rotation.signals.values);
    if size(gt_rot, 1) == 4, gt_rot = gt_rot'; end
    qw = gt_rot(1,1); qx = gt_rot(1,2); qy = gt_rot(1,3); qz = gt_rot(1,4);
    theta0 = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));

    X = [gt_pos(1,1); gt_pos(1,2); theta0; 0; 0];

    % Tight on position/heading (seeded from GT), loose on velocity (unknown)
    P = diag([0.01, 0.01, 0.01, 0.1, 0.1]);

    %% =========================================================================
    %  4. ALLOCATE OUTPUT ARRAYS
    % ==========================================================================
    num_steps  = length(imu_time);
    X_Est      = zeros(num_steps, 5);
    P_Est      = zeros(5, 5, num_steps);
    mag_idx    = 1;
    tof_idxs   = [1, 1, 1];
    prev_time  = imu_time(1);
    mag_offset = [];  % magnetic-north-to-arena-frame offset, set on first mag sample

    %% =========================================================================
    %  5. MAIN EKF LOOP
    % ==========================================================================
    for k = 1:num_steps

        dt = imu_time(k) - prev_time;
        if dt <= 0, dt = 0.001; end
        prev_time = imu_time(k);

        % -----------------------------------------------------------------------
        % 5a. PREDICTION — gyroscope dead-reckoning (no accelerometer)
        %     Yaw axis = Board X (index 1). No negation: board X points world +Z,
        %     so CCW rotation gives positive gyro X by the right-hand rule.
        % -----------------------------------------------------------------------
        omega   = gyro_scale * (gyro_data(k, 1) - gyro_bias(1));
        vx_pred = vel_decay * X(4);
        vy_pred = vel_decay * X(5);

        X(1) = X(1) + (X(4)*cos(X(3)) - X(5)*sin(X(3))) * dt;
        X(2) = X(2) + (X(4)*sin(X(3)) + X(5)*cos(X(3))) * dt;
        X(3) = wrapToPi(X(3) + omega * dt);
        X(4) = vx_pred;
        X(5) = vy_pred;

        % Linearised motion Jacobian F = d(f)/d(X)
        F      = eye(5);
        F(1,3) = (-X(4)*sin(X(3)) - X(5)*cos(X(3))) * dt;
        F(1,4) =  cos(X(3)) * dt;
        F(1,5) = -sin(X(3)) * dt;
        F(2,3) =  (X(4)*cos(X(3)) - X(5)*sin(X(3))) * dt;
        F(2,4) =  sin(X(3)) * dt;
        F(2,5) =  cos(X(3)) * dt;
        F(4,4) =  vel_decay;
        F(5,5) =  vel_decay;

        P = F * P * F' + Q;

        % -----------------------------------------------------------------------
        % 5b. MAGNETOMETER UPDATE — heading correction at ~50 Hz
        %     Horizontal axes: Board Y (index 2) and Board Z (index 3).
        %     Board X (index 1) is vertical — not used for heading.
        % -----------------------------------------------------------------------
        if mag_idx <= length(mag_time) && imu_time(k) >= mag_time(mag_idx)

            my    = (mag_data(mag_idx, 2) - mag_hard(1)) * mag_soft(1);  % Board Y
            mz    = (mag_data(mag_idx, 3) - mag_hard(2)) * mag_soft(2);  % Board Z
            z_mag = wrapToPi(atan2(mz, my));

            if isempty(mag_offset)
                mag_offset = wrapToPi(theta0 - z_mag);
            end

            inn_mag = wrapToPi((z_mag + mag_offset) - X(3));
            H_mag   = [0, 0, 1, 0, 0];
            S_mag   = H_mag * P * H_mag' + R_mag;
            K_mag   = P * H_mag' / S_mag;
            X       = X + K_mag * inn_mag;
            X(3)    = wrapToPi(X(3));

            I_KH = eye(5) - K_mag * H_mag;
            P    = I_KH * P * I_KH' + K_mag * R_mag * K_mag';

            mag_idx = mag_idx + 1;
        end

        % -----------------------------------------------------------------------
        % 5c. ToF UPDATES — position correction at ~10 Hz per sensor
        %     Ray-cast predicts expected wall distance from current pose estimate.
        %     Dual gate rejects wall-hole false readings before Kalman update.
        % -----------------------------------------------------------------------
        for i = 1:3
            if tof_idxs(i) <= length(tof_times{i}) && ...
               imu_time(k) >= tof_times{i}(tof_idxs(i))

                data = tof_raw{i}(tof_idxs(i), :);

                if data(4) == 0 && data(1) > 0.05 && data(1) < 4.0
                    [h, H] = ray_cast(X, bounds, alpha_tofs(i));

                    if h > 0 && h < 5.0
                        inn   = data(1) - h;
                        S_tof = H * P * H' + R_tof;

                        if (inn^2 / S_tof) < gate_chi2 && abs(inn) < gate_abs
                            K_tof = P * H' / S_tof;
                            X     = X + K_tof * inn;
                            X(3)  = wrapToPi(X(3));

                            I_KH = eye(5) - K_tof * H;
                            P    = I_KH * P * I_KH' + K_tof * R_tof * K_tof';
                        end
                    end
                end

                tof_idxs(i) = tof_idxs(i) + 1;
            end
        end

        X_Est(k, :)    = X';
        P_Est(:, :, k) = P;

    end % main EKF loop

end % myEKF


%% ============================================================================
%  HELPER FUNCTIONS
% =============================================================================

function [h, H] = ray_cast(X, b, alpha)
% RAY_CAST  Expected ToF distance to nearest arena wall + measurement Jacobian.
%   Casts a ray from [X(1), X(2)] in direction X(3)+alpha and returns the
%   shortest positive intersection h with the four arena walls, plus H = d(h)/d(X).

    phi = wrapToPi(X(3) + alpha);
    cp  = cos(phi);
    sp  = sin(phi);

    d = [inf, inf, inf, inf];
    if  cp >  1e-6,  d(1) = (b.x_max - X(1)) / cp; end  % +X wall
    if  cp < -1e-6,  d(2) = (b.x_min - X(1)) / cp; end  % -X wall
    if  sp >  1e-6,  d(3) = (b.y_max - X(2)) / sp; end  % +Y wall
    if  sp < -1e-6,  d(4) = (b.y_min - X(2)) / sp; end  % -Y wall

    [h, idx] = min(d);

    H = zeros(1, 5);
    if idx <= 2
        H(1) = -1 / cp;         % dh/dx
        H(3) =  h * (sp / cp);  % dh/dtheta
    else
        H(2) = -1 / sp;         % dh/dy
        H(3) = -h * (cp / sp);  % dh/dtheta
    end
end


function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end