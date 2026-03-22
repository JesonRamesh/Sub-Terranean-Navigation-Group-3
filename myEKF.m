function [X_Est, P_Est] = myEKF(out)
% myEKF - Extended Kalman Filter for 2D robot localisation inside a walled arena.
%
%   [X_Est, P_Est] = myEKF(out)
%
%   Inputs:
%       out   - Struct of logged sensor data (IMU, magnetometer, ToF, ground truth)
%   Outputs:
%       X_Est - [N x 5] state estimates [x, y, theta_world, vx, vy] (theta = GT yaw)
%       P_Est - [5 x 5 x N] state covariance matrices at each IMU timestep
%
%   Internal state X: [x; y; theta_sensor; vx; vy]. Output column 3 is theta_world.
%   vx, vy are body-frame (forward/lateral in the board Y-Z plane). Sensors fused:
%   Gyro (heading), Accel Y-Z (body velocity -> world position via θ_world),
%   Magnetometer (heading), 3x ToF (range).
%
%   BOARD ORIENTATION (confirmed by calib1_rotate.mat mag axis-pair scatter):
%   The mag Y-Z pair forms a complete circle during horizontal rotation.
%   Board X is therefore the vertical axis (world +Z).
%     Board X --> World +Z  (vertical, up)
%     Board Y --> horizontal plane axis 1
%     Board Z --> horizontal plane axis 2
%   Gyro yaw rate = gyro(:,1)  [Board X = world +Z, CCW positive]
%   Mag heading   = atan2(mag(:,3), mag(:,2))  [Board Z, Board Y]
%
%   X(3) is the sensor-plane heading (gyro + mag in the Y-Z plane). PhaseSpace
%   quaternion yaw (test_ekf yaw_gt) is the world XY heading θ with θ=0 along +X.
%   θ_world = wrapToPi(X(3) - heading_plane_offset). The mount offset
%   heading_plane_offset ≈ X(3) - θ_world is estimated from the first N GT
%   quaternion yaws and time-aligned calibrated mag headings (often near -π/2).
%   Kinematics, ToF rays, and X_Est(:,3) use θ_world; gyro/mag update X(3).

    %% =========================================================================
    %  1. DATA EXTRACTION
    % ==========================================================================
    imu_time  = out.Sensor_GYRO.time;
    gyro_data = squeeze(out.Sensor_GYRO.signals.values)';  % [N x 3] rad/s

    if isfield(out, 'Sensor_ACCEL')
        accel_time = out.Sensor_ACCEL.time(:);
        accel_data = squeeze(out.Sensor_ACCEL.signals.values)';
        if size(accel_data, 1) == 3, accel_data = accel_data'; end
    else
        accel_time = [];
        accel_data = [];
    end

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

    % ----- Accelerometer (calib2_straight.mat stationary window; paste from calibration.m)
    accel_bias = [0.0, 0.0, 0.0]; % [X, Y, Z] m/s^2 — Y,Z drive body forward/lateral velocity

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

    % Q: process noise — low on position/heading; velocity driven by noisy accel
    Q = diag([1e-4, 1e-4, 1e-7, 0.8, 0.8]);

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

    % Dead reckoning test: set false for prediction + mag only (no ToF).
    enable_tof_updates = true;

    %% =========================================================================
    %  3. FILTER INITIALISATION
    %   First N GT samples: mean position, circular-mean quaternion yaw, and
    %   heading_plane_offset = circmean(wrapToPi(z_mag - theta_quat)) so internal
    %   sensor heading matches world yaw via θ_world = wrapToPi(X(3) - offset).
    % ==========================================================================
    gt_rot = squeeze(out.GT_rotation.signals.values);
    if size(gt_rot, 1) == 4, gt_rot = gt_rot'; end

    N_init = max(1, min([10, size(gt_rot, 1), size(gt_pos, 1)]));
    if isfield(out.GT_rotation, 'time')
        gt_t = out.GT_rotation.time(:);
    elseif isfield(out.GT_position, 'time')
        gt_t = out.GT_position.time(:);
    else
        gt_t = imu_time(1:min(length(imu_time), size(gt_rot, 1)));
    end
    theta_quat_s = zeros(N_init, 1);
    delta        = zeros(N_init, 1);
    for k = 1:N_init
        qw = gt_rot(k, 1); qx = gt_rot(k, 2); qy = gt_rot(k, 3); qz = gt_rot(k, 4);
        theta_quat_s(k) = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy^2 + qz^2));
        tk = gt_t(min(k, length(gt_t)));
        [~, mag_idx_init] = min(abs(mag_time(:) - tk));
        my = (mag_data(mag_idx_init, 2) - mag_hard(1)) * mag_soft(1);
        mz = (mag_data(mag_idx_init, 3) - mag_hard(2)) * mag_soft(2);
        z_mag_k = wrapToPi(atan2(mz, my));
        delta(k) = wrapToPi(z_mag_k - theta_quat_s(k));
    end
    heading_plane_offset = atan2(mean(sin(delta)), mean(cos(delta)));
    theta_quat_mean = atan2(mean(sin(theta_quat_s)), mean(cos(theta_quat_s)));
    theta_sensor0   = wrapToPi(theta_quat_mean + heading_plane_offset);

    pos0 = mean(gt_pos(1:N_init, 1:2), 1);
    X    = [pos0(1); pos0(2); theta_sensor0; 0; 0];

    fprintf('myEKF: N_init=%d, heading_plane_offset=%.4f rad (data-est. mount offset)\n', ...
        N_init, heading_plane_offset);

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
        % 5a. PREDICTION — gyro (yaw) + horizontal accel (body Y/Z -> velocity)
        %     Yaw rate = Board X (index 1). Body forward/lateral accel on Y,Z
        %     integrate into X(4:5), then map to world with θ_world for X(1:2).
        % -----------------------------------------------------------------------
        omega = gyro_scale * (gyro_data(k, 1) - gyro_bias(1));
        tw    = wrapToPi(X(3) - heading_plane_offset);

        if ~isempty(accel_data)
            [~, ia] = min(abs(accel_time - imu_time(k)));
            ay = accel_data(ia, 2) - accel_bias(2);
            az = accel_data(ia, 3) - accel_bias(3);
            a_fwd = ay;
            a_lat = az;
        else
            a_fwd = 0;
            a_lat = 0;
        end

        vx_pred = vel_decay * (X(4) + a_fwd * dt);
        vy_pred = vel_decay * (X(5) + a_lat * dt);

        X(1) = X(1) + (vx_pred * cos(tw) - vy_pred * sin(tw)) * dt;
        X(2) = X(2) + (vx_pred * sin(tw) + vy_pred * cos(tw)) * dt;
        X(3) = wrapToPi(X(3) + omega * dt);
        X(4) = vx_pred;
        X(5) = vy_pred;

        % Linearised motion Jacobian F = d(f)/d(X); tw = X(3) - heading_plane_offset
        vd = vel_decay;
        F  = eye(5);
        F(1, 3) = (-vx_pred * sin(tw) - vy_pred * cos(tw)) * dt;
        F(1, 4) =  cos(tw) * vd * dt;
        F(1, 5) = -sin(tw) * vd * dt;
        F(2, 3) =  (vx_pred * cos(tw) - vy_pred * sin(tw)) * dt;
        F(2, 4) =  sin(tw) * vd * dt;
        F(2, 5) =  cos(tw) * vd * dt;
        F(4, 4) = vd;
        F(5, 5) = vd;

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
                mag_offset = wrapToPi(theta_sensor0 - z_mag);
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
        %     Skipped when enable_tof_updates is false (dead-reckoning test).
        % -----------------------------------------------------------------------
        if enable_tof_updates
            for i = 1:3
                if tof_idxs(i) <= length(tof_times{i}) && ...
                   imu_time(k) >= tof_times{i}(tof_idxs(i))

                    data = tof_raw{i}(tof_idxs(i), :);

                    if data(4) == 0 && data(1) > 0.05 && data(1) < 4.0
                        [h, H] = ray_cast(X, bounds, alpha_tofs(i), heading_plane_offset);

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
        end

        X_out        = X;
        X_out(3)     = wrapToPi(X(3) - heading_plane_offset);
        X_Est(k, :)  = X_out';
        P_Est(:, :, k) = P;

    end % main EKF loop

end % myEKF


%% ============================================================================
%  HELPER FUNCTIONS
% =============================================================================

function [h, H] = ray_cast(X, b, alpha, heading_plane_offset)
% RAY_CAST  Expected ToF distance to nearest arena wall + measurement Jacobian.
%   Casts a ray from [X(1), X(2)] in direction θ_world+alpha and returns the
%   shortest positive intersection h with the four arena walls, plus H = d(h)/d(X).
%   θ_world = wrapToPi(X(3) - heading_plane_offset).

    if nargin < 4, heading_plane_offset = 0; end
    phi = wrapToPi(X(3) - heading_plane_offset + alpha);
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