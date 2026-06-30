function auv_sil_controller(block)
    setup(block);
end

function setup(block)
    block.NumInputPorts  = 2;
    block.NumOutputPorts = 5;
    
    block.InputPort(1).Dimensions = 13;
    block.InputPort(1).DatatypeID = -1; 
    block.InputPort(1).DirectFeedthrough = false;
    
    block.InputPort(2).Dimensions = 4;
    block.InputPort(2).DatatypeID = 0; 
    block.InputPort(2).DirectFeedthrough = false;
    
    block.OutputPort(1).Dimensions = 8;
    block.OutputPort(1).DatatypeID = 0; 
    
    block.OutputPort(2).Dimensions = 6;
    block.OutputPort(2).DatatypeID = 0; 
    
    block.OutputPort(3).Dimensions = 6;
    block.OutputPort(3).DatatypeID = 0; 
    
    block.OutputPort(4).Dimensions = 6;
    block.OutputPort(4).DatatypeID = 0; 
    
    block.OutputPort(5).Dimensions = 6;
    block.OutputPort(5).DatatypeID = 0; 
    
    block.SampleTimes = [0.01 0]; 
    
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Update', @Update);
end

function Start(block)
    global auv_state;
    auv_state = struct();
    
    
    auv_state.q = [1; 0; 0; 0];
    auv_state.pos = zeros(3,1);
    auv_state.vel = zeros(3,1); 
    
    
    auv_state.integrals = zeros(6,1);
    auv_state.prev_err = zeros(6,1);
    
    
    auv_state.pwms = 1500 * ones(8,1);
    auv_state.target_vel = zeros(6,1);
    auv_state.smooth_target = zeros(6,1); 
    auv_state.tau = zeros(6,1);
    
    
    auv_state.Kp = [1.5, 1.5, 8.0, 0.5, 0.5, 0.8]; 
    auv_state.Ki = [0.0, 0.0, 15.0, 1.5, 1.5, 0.0];
    auv_state.Kd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    
    
    auv_state.tam = [
      0.3536  -0.3536   0.0000   0.0000   0.0000  -0.9270 ;
      0.3536   0.3536   0.0000   0.0000   0.0000   0.9270 ;
     -0.3536  -0.3536   0.0000   0.0000   0.0000   0.9270 ;
     -0.3536   0.3536   0.0000   0.0000   0.0000  -0.9270 ;
      0.0000   0.0000   0.2500   3.2258  -1.2821   0.0000 ;
      0.0000   0.0000   0.2500  -3.2258  -1.2821   0.0000 ;
      0.0000   0.0000   0.2500   3.2258   1.2821   0.0000 ;
      0.0000   0.0000   0.2500  -3.2258   1.2821   0.0000 ;
    ];
end

function Outputs(block)
    global auv_state;
    block.OutputPort(1).Data = double(auv_state.pwms);
    
    q0 = auv_state.q(1); q1 = auv_state.q(2); q2 = auv_state.q(3); q3 = auv_state.q(4);
    roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    pitch = asin(2*(q0*q2 - q3*q1));
    yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
    
    curr_pos = [auv_state.pos; roll; pitch; yaw];
    block.OutputPort(2).Data = double(curr_pos);
    
    curr_vel = [auv_state.vel; 0; 0; 0];
    block.OutputPort(3).Data = double(curr_vel);
    block.OutputPort(4).Data = double(auv_state.target_vel);
    block.OutputPort(5).Data = double(auv_state.tau);
end

function Update(block)
    global auv_state;
    dt = 0.01;
    
    
    udp_in = double(block.InputPort(2).Data);
    auv_state.target_vel = [udp_in(1); udp_in(2); udp_in(3); 0; 0; udp_in(4)];
    
    
    sensors = double(block.InputPort(1).Data);
    acc = sensors(1:3);
    gyro = sensors(4:6);
    mag = sensors(7:9);
    depth = sensors(10);
    dvl_raw = sensors(11:13);
    
    
    
    q = auv_state.q;
    w = gyro;
    dq = [1; 0.5*w(1)*dt; 0.5*w(2)*dt; 0.5*w(3)*dt];
    q_new = [
        q(1)*dq(1) - q(2)*dq(2) - q(3)*dq(3) - q(4)*dq(4);
        q(1)*dq(2) + q(2)*dq(1) + q(3)*dq(4) - q(4)*dq(3);
        q(1)*dq(3) - q(2)*dq(4) + q(3)*dq(1) + q(4)*dq(2);
        q(1)*dq(4) + q(2)*dq(3) - q(3)*dq(2) + q(4)*dq(1)
    ];
    q_new = q_new / norm(q_new);
    if q_new(1) < 0, q_new = -q_new; end
    auv_state.q = q_new;
    
    q0 = q_new(1); q1 = q_new(2); q2 = q_new(3); q3 = q_new(4);
    roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    pitch = asin(2*(q0*q2 - q3*q1));
    yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
    
    
    alpha = 0.95; 
    auv_state.vel = alpha * auv_state.vel + (1.0 - alpha) * dvl_raw;
    
    
    R = [
        1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2);
        2*(q1*q2+q0*q3),   1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
        2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1),   1-2*(q1^2+q2^2)
    ];
    v_earth = R * auv_state.vel;
    auv_state.pos(1) = auv_state.pos(1) + v_earth(1) * dt;
    auv_state.pos(2) = auv_state.pos(2) + v_earth(2) * dt;
    auv_state.pos(3) = depth;
    
    curr_vel_6dof = [auv_state.vel; gyro];
    
    
    
    for i = 1:6
        if abs(auv_state.target_vel(i)) < abs(auv_state.smooth_target(i))
            
            auv_state.smooth_target(i) = 0.5 * auv_state.smooth_target(i) + 0.5 * auv_state.target_vel(i);
        else
            
            auv_state.smooth_target(i) = 0.98 * auv_state.smooth_target(i) + 0.02 * auv_state.target_vel(i); 
        end
    end
    
    
    tau = zeros(6,1);
    
    
    tau_max = [13.0, 13.0, 18.0, 1.5, 3.5, 5.0];
    
    for i = 1:6
        desired_vel = auv_state.smooth_target(i);
        
        if i == 4
            pos_err = 0 - roll;
            if abs(pos_err) < 0.035, pos_err = 0; end
            desired_vel = 0.5 * pos_err;
        elseif i == 5
            pos_err = 0 - pitch;
            if abs(pos_err) < 0.035, pos_err = 0; end
            desired_vel = 0.5 * pos_err;
        end
        
        vel_err = desired_vel - curr_vel_6dof(i);
        
        
        
        
        if abs(auv_state.target_vel(i)) < 0.1
            auv_state.integrals(i) = auv_state.integrals(i) + vel_err * dt;
        end
        
        i_term = auv_state.Ki(i) * auv_state.integrals(i);
        
        
        if i_term > tau_max(i)
            i_term = tau_max(i);
            if auv_state.Ki(i) ~= 0, auv_state.integrals(i) = tau_max(i) / auv_state.Ki(i); end
        elseif i_term < -tau_max(i)
            i_term = -tau_max(i);
            if auv_state.Ki(i) ~= 0, auv_state.integrals(i) = -tau_max(i) / auv_state.Ki(i); end
        end
        
        d_term = auv_state.Kd(i) * (vel_err - auv_state.prev_err(i)) / dt;
        auv_state.prev_err(i) = vel_err;
        
        out = auv_state.Kp(i) * vel_err + i_term + d_term;
        
        
        if out > tau_max(i), out = tau_max(i); end
        if out < -tau_max(i), out = -tau_max(i); end
        
        tau(i) = out;
    end
    auv_state.tau = tau;
    
    
    thrusters_force = auv_state.tam * tau;
    
    
    pwms_arr = [1100, 1104, 1108, 1112, 1116, 1120, 1124, 1128, 1132, 1136, 1140, 1144, 1148, 1152, 1156, 1160, 1164, 1168, 1172, 1176, 1180, 1184, 1188, 1192, 1196, 1200, 1204, 1208, 1212, 1216, 1220, 1224, 1228, 1232, 1236, 1240, 1244, 1248, 1252, 1256, 1260, 1264, 1268, 1272, 1276, 1280, 1284, 1288, 1292, 1296, 1300, 1304, 1308, 1312, 1316, 1320, 1324, 1328, 1332, 1336, 1340, 1344, 1348, 1352, 1356, 1360, 1364, 1368, 1372, 1376, 1380, 1384, 1388, 1392, 1396, 1400, 1404, 1408, 1412, 1416, 1420, 1424, 1428, 1432, 1436, 1440, 1444, 1448, 1452, 1456, 1460, 1464, 1476, 1500, 1524, 1536, 1540, 1544, 1548, 1552, 1556, 1560, 1564, 1568, 1572, 1576, 1580, 1584, 1588, 1592, 1596, 1600, 1604, 1608, 1612, 1616, 1620, 1624, 1628, 1632, 1636, 1640, 1644, 1648, 1652, 1656, 1660, 1664, 1668, 1672, 1676, 1680, 1684, 1688, 1692, 1696, 1700, 1704, 1708, 1712, 1716, 1720, 1724, 1728, 1732, 1736, 1740, 1744, 1748, 1752, 1756, 1760, 1764, 1768, 1772, 1776, 1780, 1784, 1788, 1792, 1796, 1800, 1804, 1808, 1812, 1816, 1820, 1824, 1828, 1832, 1836, 1840, 1844, 1848, 1852, 1856, 1860, 1864, 1868, 1872, 1876, 1880, 1884, 1888, 1892, 1896];
    forces_arr = [-3.52, -3.5, -3.49, -3.45, -3.4, -3.36, -3.29, -3.25, -3.19, -3.14, -3.1, -3.06, -3.0, -2.94, -2.88, -2.85, -2.78, -2.76, -2.69, -2.64, -2.59, -2.53, -2.49, -2.45, -2.41, -2.35, -2.34, -2.26, -2.2, -2.18, -2.12, -2.05, -2.03, -1.99, -1.91, -1.89, -1.82, -1.76, -1.72, -1.68, -1.63, -1.58, -1.56, -1.52, -1.48, -1.44, -1.4, -1.37, -1.32, -1.28, -1.24, -1.19, -1.17, -1.12, -1.09, -1.05, -1.02, -0.98, -0.95, -0.92, -0.88, -0.85, -0.81, -0.77, -0.74, -0.7, -0.68, -0.65, -0.62, -0.59, -0.55, -0.52, -0.49, -0.46, -0.43, -0.4, -0.37, -0.35, -0.32, -0.29, -0.26, -0.24, -0.21, -0.19, -0.16, -0.15, -0.12, -0.1, -0.08, -0.07, -0.05, -0.03, -0.001, 0.0, 0.001, 0.05, 0.06, 0.08, 0.1, 0.12, 0.15, 0.18, 0.2, 0.23, 0.26, 0.29, 0.33, 0.36, 0.39, 0.43, 0.46, 0.5, 0.53, 0.58, 0.62, 0.64, 0.69, 0.73, 0.77, 0.83, 0.85, 0.89, 0.92, 0.97, 1.0, 1.05, 1.09, 1.14, 1.2, 1.23, 1.28, 1.32, 1.37, 1.41, 1.46, 1.51, 1.55, 1.61, 1.65, 1.71, 1.76, 1.81, 1.85, 1.91, 1.96, 2.0, 2.09, 2.12, 2.16, 2.25, 2.27, 2.34, 2.43, 2.5, 2.56, 2.64, 2.66, 2.76, 2.78, 2.88, 2.93, 2.99, 3.05, 3.13, 3.19, 3.23, 3.32, 3.36, 3.42, 3.49, 3.57, 3.62, 3.69, 3.77, 3.84, 3.92, 3.98, 4.03, 4.11, 4.15, 4.21, 4.3, 4.38, 4.42, 4.51, 4.53];
    
    for i = 1:8
        f = thrusters_force(i);
        if f > 4.53
            auv_state.pwms(i) = 1900;
        elseif f < -3.52
            auv_state.pwms(i) = 1100;
        else
            auv_state.pwms(i) = interp1(forces_arr, pwms_arr, f, 'linear');
        end
    end
end
