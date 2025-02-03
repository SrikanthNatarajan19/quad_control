function [F, M, trpy, drpy] = pid_controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

persistent gd;
persistent icnt;
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;
 
% =================== Your code starts here ===================
%% Parameter Initialization

% F = 0;
% M = [0;0;0];

    pos = qd{qn}.pos;              
    vel = qd{qn}.vel;              
    euler = qd{qn}.euler;          
    omega = qd{qn}.omega;          
    pos_des = qd{qn}.pos_des;      
    yaw_des = qd{qn}.yaw_des;      
    yawdot_des = qd{qn}.yawdot_des;
    mass = params.mass;               
    grav = params.grav;            
    I = params.I;           
    
    Kp_att = [70.0; 70.0; 140.0];  
    Kd_att = [16.0; 16.0; 16.0];  
    Kp_pos = [22.0; 22.0; 30.0];        
    Kd_pos = [8.0; 8.0; 7.2];    

    pos_error = pos_des - pos;     
    vel_error = - vel;

    acc_des = Kd_pos .* vel_error + Kp_pos .* pos_error ;

    phi_des = (1 / grav) * (acc_des(1) * sin(yaw_des) - acc_des(2) * cos(yaw_des));
    theta_des = (1 / grav) * (acc_des(1) * cos(yaw_des) + acc_des(2) * sin(yaw_des));

    euler_des = [phi_des; theta_des; yaw_des];
    euler_error = euler_des - euler;
    omega_des = [0; 0; yawdot_des];  
    omega_error = omega_des - omega;

    F = mass * (acc_des(3)+grav);
    M = I * (Kp_att .* euler_error + Kd_att .* omega_error);

% =================== Your code ends here ===================

%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

end
