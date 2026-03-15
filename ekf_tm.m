function [X_Est, P_Est, GT] = myEKF(out)
persistent acc_bias gyro_bias mag_bias

if isempty(acc_bias)
    acc_bias = [0, 0, 0];
    gyro_bias = [0, 0, 0];
    mag_bias = [0, 0, 0];
end

acc = squeeze(out.Sensor_ACCEL.signals.values)';
gyro = squeeze(out.Sensor_GYRO.signals.values)';
mag = squeeze(out.Sensor_MAG.signals.values)';

accX = acc(1) + acc_bias(1);
accY = acc(2) + acc_bias(2);
accZ = acc(3) + acc_bias(3);

gyroX = gyro(1) + gyro_bias(1);
gyroY = gyro(2) + gyro_bias(2);
gyroZ = gyro(3) + gyro_bias(3);

magX = -1E-4*mag(1) + mag_bias(1); %convert to Tesla
magY = 1E-4*mag(2) + mag_bias(2);
magZ = -1E-4*mag(3) + mag_bias(3);

% --------------------------------------------------------------
% 1) Persistent state and covariance
% --------------------------------------------------------------
persistent x P
if isempty(x)
    % Initialise position from first ground truth point
    x0 = out.GT_position.signals.values(1,1);
    y0 = out.GT_position.signals.values(1,2);
    
    x = [0; 0; 0; x0; y0; 0; 0];  % [phi;theta;psi;x;y;vx;vy]
    
    P = diag([0.1, 0.1, (180/pi)^2, ...  % attitude uncertainty
              1.0, 1.0,             ...  % position uncertainty [m²]
              0.5, 0.5]);                % velocity uncertainty [m²/s²]
end

% --------------------------------------------------------------
% 2) Known constants and sensor-noise settings
% (Use your measured std devs from Task 2!)
% --------------------------------------------------------------
dt = 0.01; % 100 Hz
g = 9.81; % gravitational acceleration [m/s^2]

% Example: local magnetic field in London (approx).
% In an "NED" coordinate system, let's say B ≈ [18.5; -2.9; 48.7] microTesla
% for an inclination ~66° down from horizontal. You can scale to Tesla or Gauss.
% We'll store it in [X; Y; Z] inertial, with Z downward.
% Adjust these numbers for your reference frame/orientation.

M_inertial = [ 19437.4e-9; 181.2E-9; 45090.4E-9 ];

% (You may need to rotate or tweak this to match your "inertial" frame choice.)
% Example process noise (Q): Tweak if needed.
% You might base these on gyro bias/drift or small angle random walk.

Q = diag([1e-3, 1e-3, 1e-3, ...   % attitude
          1e-4, 1e-4,       ...   % position (small, driven by velocity)
          1e-2, 1e-2]);            % velocity (accel integration drift)

% Example measurement noise (R): 6x6, for (a_x,a_y,a_z,m_x,m_y,m_z).
% Replace the diag entries with (sigma_acc^2, sigma_mag^2, etc.)
% from Task 2.

R = diag([ ...
 (70E-3)^2, (70E-3)^2, 0.02^2, ... % accelerometer noise
 0.005^2, 0.005^2, 0.005^2 ... % magnetometer noise
 ]);

% --------------------------------------------------------------
% 3) EKF Prediction Step
% Using continuous-time Euler-angle kinematics + Euler discretization.
% --------------------------------------------------------------
% Current state
phi_k = x(1);
theta_k = x(2);
psi_k = x(3);


% Body rates from gyro (p,q,r)
p = gyroX;
q = gyroY;
r = gyroZ;

% --- f(x,u): Euler-angle rates ---
phi_dot = p + q*sin(phi_k)*tan(theta_k) + r*cos(phi_k)*tan(theta_k);
theta_dot = q*cos(phi_k) - r*sin(phi_k);
psi_dot = (q*sin(phi_k) + r*cos(phi_k)) / cos(theta_k);

% Discrete prediction: x_pred = x + f*dt
x_pred = x + dt * [phi_dot; theta_dot; psi_dot];

% --- Compute the Jacobian F of f wrt x (3x3) ---
% Partial derivatives of [phi_dot; theta_dot; psi_dot] wrt (phi,theta,psi)
% ignoring direct psi dependency in the standard 3-2-1 equations:
F_ct = zeros(7);


% Attitude block
F_ct(1,1) = q*cos(phi_k)*tan(theta_k) - r*sin(phi_k)*tan(theta_k);
F_ct(1,2) = (q*sin(phi_k) + r*cos(phi_k)) / cos(theta_k)^2;
F_ct(2,1) = -q*sin(phi_k) - r*cos(phi_k);
F_ct(3,1) = (q*cos(phi_k) - r*sin(phi_k)) / cos(theta_k);
F_ct(3,2) = (q*sin(phi_k) + r*cos(phi_k)) * sin(theta_k)/cos(theta_k)^2;

% Position block: x_dot = vx, y_dot = vy
F_ct(4,6) = 1;   % dx/dvx
F_ct(5,7) = 1;   % dy/dvy

% Velocity block: vx,vy depend on attitude (phi,theta,psi) via Rbi ---
% Use numerical diff for dA_inertial/d(attitude) — easiest approach:
for i = 1:3
    x_shift    = x; 
    x_shift(i) = x_shift(i) + 1e-6;
    Rbi_shift  = (rotX(x_shift(1))*rotY(x_shift(2))*rotZ(x_shift(3)))';
    a_shift    = Rbi_shift * a_body - [0;0;g];
    F_ct(6,i)  = (a_shift(1) - ax_inertial) / 1e-6;
    F_ct(7,i)  = (a_shift(2) - ay_inertial) / 1e-6;
end

F = eye(7) + dt * F_ct;

% Predicted covariance
P_pred = F * P * F' + Q;

% --------------------------------------------------------------
% 4) EKF Measurement Step
% z = [accX; accY; accZ; magX; magY; magZ]
% h(x) = R(phi,theta,psi)*g_inertial + R(phi,theta,psi)*M_inertial
% --------------------------------------------------------------
% Construct measurement vector from sensor
z = [accX; accY; accZ; magX; magY; magZ];

% We'll define the inertial gravity vector as [0; 0; -g].
g_inertial = [0; 0; g];

% ---- Predict sensor readings from x_pred ----
% Build the total rotation from Euler angles
phi_p = x_pred(1);
theta_p = x_pred(2);
psi_p = x_pred(3);

% rotation matrix from inertial frame to body (3-2-1)
Rib = rotX(phi_p)*rotY(theta_p)*rotZ(psi_p);

% Predicted accelerometer output (gravity only)
acc_pred = Rib * g_inertial;

% Predicted magnetometer output
mag_pred = Rib * M_inertial;

% h(x_pred)
z_pred = [acc_pred; mag_pred];

% ---- Compute H = d(h)/d(x) (6x3) ----
% We need partial derivatives of (Rib*g_inertial) wrt phi,theta,psi
% plus partial derivatives of (Rib*M_inertial) wrt phi,theta,psi
H = zeros(6,7);

% Numerical differentiation only over attitude states (cols 1-3)
% cols 4-7 stay zero for acc/mag measurements

% We can build them by computing d/d(phi)[Rib*g_inertial], etc.
% For brevity, let's do a small numeric approximation or call a local
% function "dRdEuler(...)". Here is a simple numeric approach for demo:
eps = 1e-6;

for i = 1:3
    x_shift    = x_pred;
    x_shift(i) = x_shift(i) + eps;
    R_shift    = rotX(x_shift(1))*rotY(x_shift(2))*rotZ(x_shift(3));
    z_shift    = [R_shift*g_inertial; R_shift*M_inertial];
    H(:,i)     = (z_shift - z_pred) / eps;
end

% Residual
y = z - z_pred;

% S = H P_pred H' + R
S = H * P_pred * H' + R;

% Kalman gain
K = P_pred * H' / S;

% Updated state & covariance
x_upd = x_pred + K*y;
P_upd = (eye(3) - K*H) * P_pred;

% --------------------------------------------------------------
% 5) Save and output
% --------------------------------------------------------------
x = x_upd;
P = P_upd;

magCorrected = [-mag(1); mag(2); -mag(3)];

phi = x(1);
theta = x(2);
psi = x(3);

end

% --------------------------------------------------------------
% Local rotation helper functions
% --------------------------------------------------------------
function Rx = rotX(a)
 ca = cos(a); sa = sin(a);
 Rx = [1 0 0;
 0 ca -sa;
 0 sa ca];
end

function Ry = rotY(b)
 cb = cos(b); sb = sin(b);
 Ry = [ cb 0 sb;
 0 1 0;
 -sb 0 cb];
end

function Rz = rotZ(c)
 cc = cos(c); sc = sin(c);
 Rz = [ cc -sc 0;
 sc cc 0;
 0 0 1];
end

