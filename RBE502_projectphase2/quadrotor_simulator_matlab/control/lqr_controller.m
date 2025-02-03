function [F, M, trpy, drpy] = lqr_controller(qd, t, qn, params, trajhandle)
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

%% Parameter Initialization
if ~isempty(t)
desired_state = trajhandle(t, qn);
end

    F = 0;
    M = [0;0;0];

    g = params.grav;
    m = params.mass;
    I = params.I; 
    dt = 0.001; 

    x = qd{qn}.pos(1);
    y = qd{qn}.pos(2);
    z = qd{qn}.pos(3);
    xdot = qd{qn}.vel(1);
    ydot = qd{qn}.vel(2);
    zdot = qd{qn}.vel(3);
    phi = qd{qn}.euler(1);
    theta = qd{qn}.euler(2);
    psi = qd{qn}.euler(3);
    p = qd{qn}.omega(1);
    q = qd{qn}.omega(2);
    r = qd{qn}.omega(3);

    x_des = desired_state.pos(1);
    y_des = desired_state.pos(2);
    z_des = desired_state.pos(3);
    xdot_des = desired_state.vel(1);
    ydot_des = desired_state.vel(2);
    zdot_des = desired_state.vel(3);
    phi_des = 0;
    theta_des = 0;
    psi_des = desired_state.yaw;
    p_des = 0;
    q_des = 0;
    r_des = 0;

    %Linearized using Jacobians
    A = zeros(12, 12);
    B = zeros(12, 4);
    
    A(1,4) = 1;
    A(2,5) = 1;
    A(3,6) = 1;
    A(8,11) = 1;
    
    A(4,7) = g * sin(psi); 
    A(4,8) = g * cos(psi);
    A(4,9) = (-1 * g * theta * sin(psi)) + (g * phi * sin(psi));
    A(5,7) = -1 * g * cos(psi);
    A(5,8) = g * sin(psi);
    A(5,9) = (g * theta * cos(psi)) + (g * phi * sin(psi));
    
    
    A(7,8) = -1 * p * sin(theta) + r * cos(theta);
    A(7,10) = cos(theta);
    A(7,12) = sin(theta);

    A(8,7) =(p * sin(theta) * sec(phi)^2) - (r * cos(theta) * sec(phi)^2);
    A(8,8) = (p * cos(theta) * tan(phi)) + (r * sin(theta) * tan(phi));
    A(8,10)= sin(theta) * tan(phi);
    A(8,12) = -1 * cos(theta) * tan(phi);

    A(9,7) = (-1 * sin(theta) * tan(phi) * sec(phi) * p) + (cos(theta) * tan(phi) * sec(phi) * r);
    A(9,8) = (-1 * cos(theta) * p / cos(phi)) + (-1 * sin(theta) * r / cos(phi));
    A(9,10) = -1 * sin(theta) / cos(phi);
    A(9,12) = cos(theta) / cos(phi);

   
    B(6,1) = 1 / m;          
    B(10,2) = 1 / I(1,1);    
    B(11,3) = 1 / I(2,2);    
    B(12,4) = 1 / I(3,3);   

    %discretized A and B matrices
    A_d = eye(12) + A * dt;
    B_d = B * dt;

    Q=diag([30,30,30,5,5,15,0.01,0.01,5,0.01,0.01,0.01]);
    R=diag([0.02,0.2,0.2,0.2]);

   
    %K = lqr(A_d, B_d, Q, R);
    K = dlqr(A_d, B_d,Q,R);

   
   
    X = [x; y; z; xdot; ydot; zdot; phi; theta; psi; p; q; r];
    X_des = [x_des; y_des; z_des; xdot_des; ydot_des; zdot_des; phi_des; theta_des; psi_des; p_des; q_des; r_des];

   
    E = X - X_des;

   
    u = -K * E;
       
    
    F = u(1);
    M = u(2:4);


%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0,       0,         0];

end
