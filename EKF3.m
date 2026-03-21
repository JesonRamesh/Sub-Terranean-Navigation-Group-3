function [X_Est, P_Est] = EKF3(out)
    %% 1. Data Extraction & Ground Truth Initialization [cite: 51, 52]
    imu_time = out.Sensor_GYRO.time;
    gyro_data = squeeze(out.Sensor_GYRO.signals.values)';
    accel_data = squeeze(out.Sensor_ACCEL.signals.values)';
    
    mag_time = out.Sensor_MAG.time;
    mag_data = squeeze(out.Sensor_MAG.signals.values)';
    
    tof_times = {out.Sensor_ToF1.time, out.Sensor_ToF2.time, out.Sensor_ToF3.time};
    tof_raw = {squeeze(out.Sensor_ToF1.signals.values), ...
               squeeze(out.Sensor_ToF2.signals.values), ...
               squeeze(out.Sensor_ToF3.signals.values)};
    
    for i = 1:3
        if size(tof_raw{i}, 1) == 4, tof_raw{i} = tof_raw{i}'; end
    end
    
    gt_pos = squeeze(out.GT_position.signals.values);
    if size(gt_pos, 1) == 3, gt_pos = gt_pos'; end

    %% 2. Online Calibration (First 0.5s)
    static_idx = imu_time < (imu_time(1) + 0.5);
    gyro_bias = mean(gyro_data(static_idx, :), 1);
    accel_static = mean(accel_data(static_idx, :), 1); %#ok<NASGU>
    
    % Magnetometer calibration from calib1_rotate dataset
    mag_hard = [-3.705e-05, 4.415e-05]; 
    mag_soft = [1.0958, 0.9196];

    %% 3. Filter Initialization [cite: 48, 49, 50]
    gt_rot = squeeze(out.GT_rotation.signals.values);
    if size(gt_rot, 1) == 4, gt_rot = gt_rot'; end
    qw = gt_rot(1,1); qx = gt_rot(1,2); qy = gt_rot(1,3); qz = gt_rot(1,4);
    theta0 = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));

    % State: [x; y; theta; vx; vy]
    X = [gt_pos(1,1); gt_pos(1,2); theta0; 0; 0];
    P = diag([0.01, 0.01, 0.01, 0.1, 0.1]);

    Q = diag([1e-4, 1e-4, 1e-7, 0.5, 0.5]);

    % R_tof: tighter than before — we trust ToF more to anchor position.
    % The systematic drift right/down suggests velocity is carrying the
    % estimate away between ToF corrections; tighter R_tof = stronger pull back.
    R_tof = 0.003;   % (0.03m)^2 — tighter than original 0.005

    R_mag = 0.5;      % unchanged

    % Gate: chi2(0.99,1) + absolute cap
    gate_threshold = 6.63;
    gate_abs = 0.45;   % slightly tighter than before to be more aggressive about outliers

    % Velocity decay — faster decay (tau~0.3s) so residual velocity
    % from the previous motion segment doesn't carry the estimate too far
    % before the next ToF correction arrives.
    vel_decay = 0.975; % exp(-0.01/0.3) at ~100Hz

    % Arena bounds
    bounds = struct('x_max', 1.2, 'x_min', -1.2, 'y_max', 1.65, 'y_min', -1.65);
    alpha_tofs = [0, -pi/2, pi/2]; % forward, left, right

    num_steps = length(imu_time);
    X_Est = zeros(num_steps, 5);
    P_Est = zeros(5, 5, num_steps);
    
    mag_idx = 1; 
    tof_idxs = [1, 1, 1];
    prev_time = imu_time(1);

    %% 4. Main EKF Loop
    for k = 1:num_steps
        dt = imu_time(k) - prev_time;
        if dt <= 0, dt = 0.001; end
        prev_time = imu_time(k);

        % --- PREDICTION ---
        omega = 1.18*(gyro_data(k, 1) - gyro_bias(1));

        % Velocity decays toward zero; ToF updates inject corrections.
        % No accel integration — avoids double-integration drift.
        vx_new = vel_decay * X(4);
        vy_new = vel_decay * X(5);

        X(1) = X(1) + (X(4)*cos(X(3)) - X(5)*sin(X(3)))*dt;
        X(2) = X(2) + (X(4)*sin(X(3)) + X(5)*cos(X(3)))*dt;
        X(3) = -wrapToPi(X(3) + omega*dt);
        X(4) = vx_new;
        X(5) = vy_new;

        % Jacobian F
        F = eye(5);
        F(1,3) = (-X(4)*sin(X(3)) - X(5)*cos(X(3)))*dt;
        F(1,4) = cos(X(3))*dt; F(1,5) = -sin(X(3))*dt;
        F(2,3) = -(X(4)*cos(X(3)) - X(5)*sin(X(3)))*dt;
        F(2,4) = sin(X(3))*dt; F(2,5) = cos(X(3))*dt;
        F(4,4) = vel_decay;
        F(5,5) = vel_decay;
        
        P = F * P * F' + Q;

        % --- UPDATE: Magnetometer --- unchanged
        if mag_idx <= length(mag_time) && imu_time(k) >= mag_time(mag_idx)
            my = (mag_data(mag_idx, 2) - mag_hard(1)) * mag_soft(1);
            mz = (mag_data(mag_idx, 3) - mag_hard(2)) * mag_soft(2);
            z_mag = wrapToPi(atan2(mz, my)); 
            
            if mag_idx == 1, mag_offset = wrapToPi(theta0 - z_mag); end
            inn_mag = wrapToPi((z_mag + mag_offset) - X(3));
            
            H_mag = [0, 0, 1, 0, 0];
            S_mag = H_mag * P * H_mag' + R_mag;
            K_mag = P * H_mag' / S_mag;
            X = X + K_mag * inn_mag;
            P = (eye(5) - K_mag*H_mag)*P*(eye(5) - K_mag*H_mag)' + K_mag*R_mag*K_mag';
            mag_idx = mag_idx + 1;
        end

        % --- UPDATE: ToF Sensors ---
        for i = 1:3
            if tof_idxs(i) <= length(tof_times{i}) && imu_time(k) >= tof_times{i}(tof_idxs(i))
                data = tof_raw{i}(tof_idxs(i), :);
                if data(4) == 0 && data(1) > 0.05 && data(1) < 4.0
                    [h, H] = ray_cast(X, bounds, alpha_tofs(i));
                    if h > 0 && h < 5.0
                        inn = data(1) - h;
                        S_tof = H * P * H' + R_tof;
                        if (inn^2 / S_tof) < gate_threshold && abs(inn) < gate_abs
                            K_tof = P * H' / S_tof;
                            X = X + K_tof * inn;
                            X(3) = wrapToPi(X(3));
                            P = (eye(5) - K_tof*H)*P*(eye(5) - K_tof*H)' + K_tof*R_tof*K_tof';
                        end
                    end
                end
                tof_idxs(i) = tof_idxs(i) + 1;
            end
        end

        X_Est(k,:) = X';
        P_Est(:,:,k) = P;
    end
end

%% Helper Functions — unchanged from original
function [h, H] = ray_cast(X, b, alpha)
    phi = wrapToPi(X(3) + alpha);
    cp = cos(phi); sp = sin(phi);
    
    d = [inf, inf, inf, inf];
    if cp > 1e-6,  d(1) = (b.x_max - X(1))/cp; end % Right Wall
    if cp < -1e-6, d(2) = (b.x_min - X(1))/cp; end % Left Wall
    if sp > 1e-6,  d(3) = (b.y_max - X(2))/sp; end % Top Wall
    if sp < -1e-6, d(4) = (b.y_min - X(2))/sp; end % Bottom Wall
    
    [h, idx] = min(d);
    H = zeros(1,5);
    
    if idx <= 2
        H(1) = -1/cp; 
        H(3) = h * (sp/cp); 
    else
        H(2) = -1/sp; 
        H(3) = -h * (cp/sp); 
    end
end

function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end