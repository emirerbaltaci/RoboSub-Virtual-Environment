function auv_sil_controller(block)
    setup(block);
end

function setup(block)
    block.NumInputPorts  = 2;
    block.NumOutputPorts = 5;
    
    % Input 1: Sensors (13 elements now, including DVL!)
    block.InputPort(1).Dimensions = 13;
    block.InputPort(1).DatatypeID = -1; % Inherit
    block.InputPort(1).DirectFeedthrough = false;
    
    % Input 2: UDP Command (4x1 vector: [X, Y, Z, Yaw])
    block.InputPort(2).Dimensions = 4;
    block.InputPort(2).DatatypeID = 0; % double
    block.InputPort(2).DirectFeedthrough = false;
    
    % Output 1: PWMs (8 elements)
    block.OutputPort(1).Dimensions = 8;
    block.OutputPort(1).DatatypeID = 0; % double
    
    % Output 2: Current Position (6 elements)
    block.OutputPort(2).Dimensions = 6;
    block.OutputPort(2).DatatypeID = 0; 
    
    % Output 3: Current Velocity (6 elements)
    block.OutputPort(3).Dimensions = 6;
    block.OutputPort(3).DatatypeID = 0; 
    
    % Output 4: Target Velocity (6 elements)
    block.OutputPort(4).Dimensions = 6;
    block.OutputPort(4).DatatypeID = 0; 
    
    % Output 5: Tau / Commanded Thrusts (6 elements)
    block.OutputPort(5).Dimensions = 6;
    block.OutputPort(5).DatatypeID = 0; 
    
    block.SampleTimes = [0.01 0]; % 100Hz
    
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Update', @Update);
end

function Start(block)
    global sil_eskf;
    global sil_pid;
    
    % Initialize ESKF
    sil_eskf = struct();
    sil_eskf.q = [1; 0; 0; 0];
    sil_eskf.pos = zeros(3,1);
    sil_eskf.vel = zeros(3,1);
    sil_eskf.ab = zeros(3,1);
    sil_eskf.gb = zeros(3,1);
    sil_eskf.me = zeros(3,1);
    sil_eskf.mb = zeros(3,1);
    sil_eskf.curr_pos = zeros(6,1);
    sil_eskf.curr_vel = zeros(6,1);
    sil_eskf.is_mag_init = false;
    
    P_diag = [1e-2*ones(3,1); 1e-3*ones(3,1); 1e-1*ones(3,1); 1e-2*ones(3,1); 1e-3*ones(3,1); 1e-1*ones(3,1); 1e-2*ones(3,1)];
    sil_eskf.P = diag(P_diag);
    
    sil_eskf.Q = diag([1e-6*ones(3,1); 1e-7*ones(3,1); 1e-4*ones(3,1); 1e-7*ones(3,1); 1e-7*ones(3,1); 1e-7*ones(3,1); 1e-7*ones(3,1)]);
    sil_eskf.R_mag = diag([0.05, 0.05, 0.05]);
    sil_eskf.R_baro = 0.1;
    
    % Initialize PID
    sil_pid = struct();
    sil_pid.target_vel = zeros(6,1);
    
    sil_pid.integrals_vel = zeros(6,1);
    sil_pid.pwms = 1500 * ones(8,1);
    sil_pid.tau = zeros(6,1);
    
    % Kp, Ki, Kd
    sil_pid.Kp_vel = [12.0, 12.0, 18.0, 0.0, 0.0, 8.0];
    sil_pid.Ki_vel = [1.0, 1.0, 5.0, 0.0, 0.0, 0.2];
    sil_pid.Kd_vel = [0.01, 0.01, 0.01, 0.0, 0.0, 0.01];
                  
    sil_pid.tam = [
      0.3536   0.3536  -0.0000   0.0000   0.0000   0.5051 ;
      0.3536  -0.3536   0.0000  -0.0000   0.0000  -0.5051 ;
     -0.3536   0.3536  -0.0000   0.0000   0.0000  -0.5051 ;
     -0.3536  -0.3536   0.0000  -0.0000   0.0000   0.5051 ;
      0.0000   0.0000   0.2500  -1.2500  -1.0000  -0.0000 ;
      0.0000   0.0000   0.2500   1.2500  -1.0000  -0.0000 ;
      0.0000   0.0000   0.2500  -1.2500   1.0000  -0.0000 ;
      0.0000   0.0000   0.2500   1.2500   1.0000  -0.0000 ;
    ];
end

function Outputs(block)
    global sil_eskf;
    global sil_pid;
    
    % Break Algebraic Loop: Outputs only depend on previously stored state
    block.OutputPort(1).Data = double(sil_pid.pwms);
    block.OutputPort(2).Data = double(sil_eskf.curr_pos);
    block.OutputPort(3).Data = double(sil_eskf.curr_vel);
    block.OutputPort(4).Data = double(sil_pid.target_vel);
    block.OutputPort(5).Data = double(sil_pid.tau);
end

function Update(block)
    global sil_eskf;
    global sil_pid;
    
    dt = 0.01;
    
    % --- 1. Commands ---
    % UDP Receive gives 4x1 vector: [X, Y, Z, Yaw]
    udp_in = double(block.InputPort(2).Data);
    
    % Map to [Surge, Sway, Heave, Roll, Pitch, Yaw]
    % Note: Roll and Pitch are forced to 0.
    sil_pid.target_vel = [udp_in(1); udp_in(2); udp_in(3); 0; 0; udp_in(4)];
    
    % --- 2. Sensors ---
    sensors = double(block.InputPort(1).Data);
    
    % The AUV Dynamics and our ESKF both use SAE J670 NED (X-forward, Y-right, Z-down).
    % The raw sensor bus is already in NED! No axis inversion is needed.
    
    acc = sensors(1:3) - sil_eskf.ab;
    gyro = sensors(4:6) - sil_eskf.gb;
    mag = sensors(7:9);
    
    if ~sil_eskf.is_mag_init
        % Automatically adopt whatever units/strength the physics engine uses!
        sil_eskf.me = mag;
        sil_eskf.is_mag_init = true;
    end
    
    depth = sensors(10);
    
    % --- 3. ESKF RK4 Nominal Update ---
    x = [sil_eskf.q; sil_eskf.pos; sil_eskf.vel];
    
    k1 = get_derivative(x, acc, gyro);
    xt = x + 0.5 * dt * k1;
    k2 = get_derivative(xt, acc, gyro);
    xt = x + 0.5 * dt * k2;
    k3 = get_derivative(xt, acc, gyro);
    xt = x + dt * k3;
    k4 = get_derivative(xt, acc, gyro);
    
    x = x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
    
    q_norm = norm(x(1:4));
    if q_norm > 1e-12
        x(1:4) = x(1:4) / q_norm;
    end
    
    sil_eskf.q = x(1:4);
    sil_eskf.pos = x(5:7);
    sil_eskf.vel = x(8:10);
    
    % --- 4. ESKF Covariance Predict ---
    R = quat2rotm(sil_eskf.q);
    F = eye(21);
    
    F(1, 2) = gyro(3) * dt;
    F(1, 3) = -gyro(2) * dt;
    F(2, 1) = -gyro(3) * dt;
    F(2, 3) = gyro(1) * dt;
    F(3, 1) = gyro(2) * dt;
    F(3, 2) = -gyro(1) * dt;
    
    F(1:3, 13:15) = -eye(3) * dt;
    F(4:6, 7:9) = eye(3) * dt;
    
    ax = acc(1)*dt; ay = acc(2)*dt; az = acc(3)*dt;
    F(7, 1) = -(R(1,2)*az - R(1,3)*ay);
    F(7, 2) = -(R(1,3)*ax - R(1,1)*az);
    F(7, 3) = -(R(1,1)*ay - R(1,2)*ax);
    F(8, 1) = -(R(2,2)*az - R(2,3)*ay);
    F(8, 2) = -(R(2,3)*ax - R(2,1)*az);
    F(8, 3) = -(R(2,1)*ay - R(2,2)*ax);
    F(9, 1) = -(R(3,2)*az - R(3,3)*ay);
    F(9, 2) = -(R(3,3)*ax - R(3,1)*az);
    F(9, 3) = -(R(3,1)*ay - R(3,2)*ax);
    
    F(7:9, 10:12) = -R * dt;
    
    sil_eskf.P = F * sil_eskf.P * F' + sil_eskf.Q * (dt * dt);
    
    % --- 5. ESKF Mag Update ---
    m_hat = R * sil_eskf.me + sil_eskf.mb;
    mx = m_hat(1) - sil_eskf.mb(1);
    my = m_hat(2) - sil_eskf.mb(2);
    mz = m_hat(3) - sil_eskf.mb(3);
    
    H = zeros(3, 21);
    H(1, 2) = -mz; H(1, 3) = my;
    H(2, 1) = mz; H(2, 3) = -mx;
    H(3, 1) = -my; H(3, 2) = mx;
    H(1:3, 16:18) = R;
    H(1:3, 19:21) = eye(3);
    
    y_mag = mag - m_hat;
    S = H * sil_eskf.P * H' + sil_eskf.R_mag;
    K = sil_eskf.P * H' / S;
    dx = K * y_mag;
    sil_eskf.P = (eye(21) - K * H) * sil_eskf.P;
    inject_error(dx);
    
    % --- 6. ESKF Baro Update ---
    H_baro = zeros(1, 21);
    H_baro(6) = 1;
    y_baro = depth - sil_eskf.pos(3);
    S_baro = H_baro * sil_eskf.P * H_baro' + sil_eskf.R_baro;
    K_baro = sil_eskf.P * H_baro' / S_baro;
    dx_baro = K_baro * y_baro;
    dx_baro = zeros(21, 1); % DISABLED BARO INJECTION
    sil_eskf.P = (eye(21) - K_baro * H_baro) * sil_eskf.P;
    inject_error(dx_baro);
    
    % --- 7. PID Controller ---
    q0 = sil_eskf.q(1); q1 = sil_eskf.q(2); q2 = sil_eskf.q(3); q3 = sil_eskf.q(4);
    roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    pitch = asin(2*(q0*q2 - q3*q1));
    yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
    
    % DVL gives us perfect velocity! No more IMU drift!
    % If DVL is Body Frame:
    v_body = sensors(11:13);
    
    % If DVL is Earth Frame, uncomment this instead:
    % R_curr = quat2rotm(sil_eskf.q);
    % v_body = R_curr' * sensors(11:13);
    
    curr_pos = [sil_eskf.pos; roll; pitch; yaw];
    curr_vel = [v_body; sensors(4); sensors(5); sensors(6)];
    sil_eskf.curr_pos = curr_pos;
    sil_eskf.curr_vel = curr_vel;
    
    tau = zeros(6,1);
    for i = 1:6
        vel_err = sil_pid.target_vel(i) - curr_vel(i);
        
        % Deadband against DVL/IMU noise
        if abs(vel_err) < 0.03
            vel_err = 0;
            
            % If we want to completely stop, bleed off any "trapped" integral thrust!
            % We skip Heave (i=3) because it MUST hold its integral to fight positive buoyancy.
            if sil_pid.target_vel(i) == 0 && i ~= 3
                sil_pid.integrals_vel(i) = sil_pid.integrals_vel(i) * 0.95;
            end
        end
        
        % Prevent Integral Windup
        sil_pid.integrals_vel(i) = sil_pid.integrals_vel(i) + vel_err * dt;
        
        max_i = 20.0;
        if i == 3
            max_i = 45.0; % Heave needs ~39 to fight 1kg buoyancy, no need to wind up to 80
        elseif i == 6
            max_i = 5.0;  % Yaw only fights rotational drag, keep windup tiny
        end
        
        i_term = sil_pid.Ki_vel(i) * sil_pid.integrals_vel(i);
        if i_term > max_i
            i_term = max_i;
            sil_pid.integrals_vel(i) = max_i / sil_pid.Ki_vel(i);
        elseif i_term < -max_i
            i_term = -max_i;
            sil_pid.integrals_vel(i) = -max_i / sil_pid.Ki_vel(i);
        end
        
        out = sil_pid.Kp_vel(i) * vel_err + i_term;
        
        % Cap total output
        if out > 200.0, out = 200.0; end
        if out < -200.0, out = -200.0; end
        
        tau(i) = out;
    end
    
    % --- 8. Thruster Allocation ---
    thrusters_force = sil_pid.tam * tau;
    
    % Desaturate
    for i = 0:1
        forces = thrusters_force((i*4 + 1):(i*4 + 4));
        max_frac = 1.0;
        for j = 1:4
            f = forces(j);
            if f > 0.0
                frac = f / 4.68;
            else
                frac = f / -3.52;
            end
            if frac > max_frac, max_frac = frac; end
        end
        if max_frac > 1.0
            thrusters_force((i*4 + 1):(i*4 + 4)) = forces / max_frac;
        end
    end
    
    % --- 9. PWM Calc ---
    pwms = zeros(8,1);
    for i = 1:8
        f = thrusters_force(i);
        if f == 0
            pwms(i) = 1500;
        elseif f > 0
            fwd_th = [0.0, 0.73, 1.13, 1.69, 2.54, 3.15, 3.76, 4.43, 4.68];
            fwd_pwm = [1525, 1650, 1700, 1750, 1800, 1850, 1900, 1950, 2000];
            if f >= 4.68
                pwms(i) = 2000;
            else
                for k = 1:8
                    if f >= fwd_th(k) && f <= fwd_th(k+1)
                        ratio = (f - fwd_th(k)) / (fwd_th(k+1) - fwd_th(k));
                        pwms(i) = fwd_pwm(k) + ratio * (fwd_pwm(k+1) - fwd_pwm(k));
                        break;
                    end
                end
            end
        else
            rev_th = [0.0, -0.74, -1.05, -1.37, -1.86, -2.11, -2.75, -3.06, -3.52];
            rev_pwm = [1475, 1350, 1300, 1250, 1200, 1150, 1100, 1050, 1000];
            if f <= -3.52
                pwms(i) = 1000;
            else
                for k = 1:8
                    if f <= rev_th(k) && f >= rev_th(k+1)
                        ratio = (f - rev_th(k)) / (rev_th(k+1) - rev_th(k));
                        pwms(i) = rev_pwm(k) + ratio * (rev_pwm(k+1) - rev_pwm(k));
                        break;
                    end
                end
            end
        end
    end
    
    sil_pid.tau = tau;
    sil_pid.pwms = pwms;
end

function ddt = get_derivative(x, a, w)
    q = x(1:4);
    v = x(8:10);
    
    ddt = zeros(10,1);
    ddt(1) = 0.5 * (-q(2)*w(1) - q(3)*w(2) - q(4)*w(3));
    ddt(2) = 0.5 * ( q(1)*w(1) + q(3)*w(3) - q(4)*w(2));
    ddt(3) = 0.5 * ( q(1)*w(2) - q(2)*w(3) + q(4)*w(1));
    ddt(4) = 0.5 * ( q(1)*w(3) + q(2)*w(2) - q(3)*w(1));
    
    ddt(5) = v(1);
    ddt(6) = v(2);
    ddt(7) = v(3);
    
    qw=q(1); qx=q(2); qy=q(3); qz=q(4);
    R0 = 1-2*(qy*qy+qz*qz); R1 = 2*(qx*qy-qw*qz);   R2 = 2*(qx*qz+qw*qy);
    R3 = 2*(qx*qy+qw*qz);   R4 = 1-2*(qx*qx+qz*qz); R5 = 2*(qy*qz-qw*qx);
    R6 = 2*(qx*qz-qw*qy);   R7 = 2*(qy*qz+qw*qx);   R8 = 1-2*(qx*qx+qy*qy);
    
    ddt(8) = R0*a(1) + R1*a(2) + R2*a(3);
    ddt(9) = R3*a(1) + R4*a(2) + R5*a(3);
    ddt(10) = R6*a(1) + R7*a(2) + R8*a(3) + 9.80665;
end

function R = quat2rotm(q)
    qw=q(1); qx=q(2); qy=q(3); qz=q(4);
    R = [
        1-2*(qy*qy+qz*qz), 2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy);
        2*(qx*qy+qw*qz),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx);
        2*(qx*qz-qw*qy),   2*(qy*qz+qw*qx),   1-2*(qx*qx+qy*qy)
    ];
end

function inject_error(dx)
    global sil_eskf;
    sil_eskf.pos = sil_eskf.pos + dx(4:6);
    sil_eskf.vel = sil_eskf.vel + dx(7:9);
    sil_eskf.ab  = sil_eskf.ab  + dx(10:12);
    sil_eskf.gb  = sil_eskf.gb  + dx(13:15);
    sil_eskf.me  = sil_eskf.me  + dx(16:18);
    sil_eskf.mb  = sil_eskf.mb  + dx(19:21);
    
    dq = [1; 0.5*dx(1); 0.5*dx(2); 0.5*dx(3)];
    
    q1 = sil_eskf.q;
    q2 = dq;
    q_new = [
        q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
        q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
        q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);
        q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1)
    ];
    
    q_norm = norm(q_new);
    if q_norm > 1e-12
        q_new = q_new / q_norm;
    end
    if q_new(1) < 0
        q_new = -q_new;
    end
    sil_eskf.q = q_new;
end
