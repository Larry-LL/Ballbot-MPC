%% Simplified 2D Model

% Because m_k = 2.556kg and m_a = 2.7kg which are much greater than
% m_w = 0.08kg. We can neglect the energy of Omniwheels but still use the
% Euler-Lagrange Equation to get Mathematical Model.
clc,clear

% clc
%% Define the symbolic variables
syms t g L m_k m_a I_k I_a_xy I_a_z R_w r_k theta_x(t) theta_y(t) theta_z(t) phi_x(t) phi_y(t) phi_z(t) psi_dot1 psi_dot2 psi_dot3 T_YZ T_XZ T_XY theta_x_dot phi_x_dot

% global alpha beta1 beta2 beta3;
% alpha = 45;
% beta1 = 0;
% beta2 = 120;
% beta3 = 240;
% % q_XZ = [theta_y(t); phi_y(t)];
% % q_XZ_dot = diff(q_XZ,t);
% % q_XZ_ddt = diff(q_XZ_dot,t);

q_YZ = [theta_x(t); phi_x(t)];
q_YZ_dot = diff(q_YZ,t);
q_YZ_ddt = diff(q_YZ_dot,t);

% % q_XY = [theta_z(t); phi_z(t)];
% % q_XY_dot = diff(q_XY,t);

%% Brief Explanations for XZ_Plane and YZ_Plane 

% Because in 2D Model, the XZ_Plane and YZ_Plane are decoupling, which
% means they should be controlled by 2 different controller without any
% connections.

%% Energy of the Ball
% The energy of the ball consists of 4 parts:
% real_phi = q_YZ(1) + q_YZ(2);
% real_phi_dot = diff(real_phi, t);

% 1. Rotational Energy
YZ_R_Ball = 1/2 * I_k * (q_YZ_dot(2))^2;

% 2. Tranlational Energy
YZ_T_Ball = 1/2 * m_k * (q_YZ_dot(2) * r_k)^2;

% 3. Potential Energy
YZ_P_Ball = 0;

% 4. XY_Plane's Rotational Energy(Probably needed if we add phi_z for ball)

%% Energy of the Body
% The energy of the Body consists of 4 parts:

YZ_R_Body = 1/2 * (I_a_xy + m_a*L^2) * (q_YZ_dot(1))^2;         % 1. Rotational Energy (Rotate around center of the ball)

YZ_T_Body = simplify(1/2 * m_a * (q_YZ_dot(2)*r_k + (q_YZ_dot(1))*L*cos(q_YZ(1)))^2  +  1/2 * m_a * ((q_YZ_dot(1))*L*sin(q_YZ(1)))^2);       % 2. Tranlational Energy

YZ_P_Body = m_a * g * L * cos(q_YZ(1));         % 3. Potential Energy

% 4. XY_Plane's Rotational Energy

%% Euler-Lagrange-Equation of YZ_Plane 
K_YZ = YZ_R_Ball + YZ_T_Ball + YZ_R_Body + YZ_T_Body;               % Total kinetic energy
P_YZ = YZ_P_Ball + YZ_P_Body;               % Total potential energy
Lagrange_YZ = K_YZ - P_YZ;              % Get the Lagrange Equation

dL_dq_YZ = (jacobian(Lagrange_YZ, q_YZ)).';         % Lagrange energy about position
dL_d_qdt = jacobian(Lagrange_YZ, q_YZ_dot).'; 
dL_dt = (diff(dL_d_qdt, t));                      % Lagrange energy about speed
YZ_eq = dL_dt - dL_dq_YZ;

Q = [0; T_YZ];                  % Control input (Torque)

M_YZ = simplify(jacobian(YZ_eq, q_YZ_ddt))
inv_M_YZ = inv(M_YZ);
Rest = simplify(Q - YZ_eq + M_YZ*q_YZ_ddt);
YZ_state_qddt = inv_M_YZ * Rest;

%% YZ Plane Linearizaiton
x = [q_YZ; q_YZ_dot];
x_dot = [q_YZ_dot; YZ_state_qddt];
u = T_YZ;
YZ_A = simplify(jacobian(x_dot, x));
YZ_B = simplify(jacobian(x_dot, T_YZ));

% Substitute the "diff terms" for Matlab requirements
symbolic_terms = [diff(theta_x(t), t), diff(phi_x(t), t)];
symbolid_values = [theta_x_dot, phi_x_dot];
YZ_A = vpa(subs(YZ_A, symbolic_terms, symbolid_values));
YZ_B = vpa(subs(YZ_B, symbolic_terms, symbolid_values));

% YZ_terms = [g, L, m_k, m_a, I_k, I_a_xy, r_k, theta_x(t), theta_x_dot, T_YZ];
% YZ_values = [9.81, 0.236, 2.558, 2.782, 0.0171, 0.055, 0.1016, 0.0, 0.0, 0.0];
% YZ_A = double(subs(YZ_A, YZ_terms, YZ_values))           % Get the YZ_A matrix's result
% YZ_B = double(subs(YZ_B, YZ_terms, YZ_values)) 
YZ_terms = [g, L, m_k, m_a, I_k, I_a_xy, r_k ];
YZ_values = [9.81, 0.236, 2.558, 2.782, 0.0171, 0.055, 0.1016];
YZ_A = vpa(simplify(subs(YZ_A, YZ_terms, YZ_values)),4)          % Get the YZ_A matrix's result
YZ_B = vpa(simplify(subs(YZ_B, YZ_terms, YZ_values)),4)           % Get the YZ_B matrix's result

YZ_Q = diag([8000; 0.001; 2000; 0.001]);
YZ_R = 40;
YZ_K = lqr(YZ_A, YZ_B, YZ_Q, YZ_R)

%% Linear Simulation Test

% A_cl = YZ_A - YZ_B * YZ_K;  
% B_cl = YZ_B;             
% C_cl = eye(size(A_cl));
% D_cl = zeros(size(C_cl, 1), size(B_cl, 2)); 
% 
% sys_cl = ss(A_cl, B_cl, C_cl, D_cl);
% step(sys_cl);
% 
% 
% x0 = [0.173; 0.0; 0.173; 0.0];
% u_esti = YZ_K * x0
% t_span = [0, 30];
% [T, X] = ode15s(@(t,x)self_balance(t, x, YZ_A, YZ_B, YZ_K), t_span, x0);



%% Things might be useful
% % XZ_R_Ball = 1/2 * I_k * (q_dot(5))^2;

% % XZ_T_Ball = 1/2 * m_k * (q_dot(5) * r_k)^2;

% % XZ_P_Ball = 0;

% % XZ_R_Body = 1/2 * PA_I_a * (q_dot(2))^2;

% % XZ_T_Body = 1/2 * m_a * (q_dot(5) * r_k)^2;

% % XZ_P_Body = m_a * g * L * cos(q(2));

% Convert phi into world coordinate for calculations
% YZ_real_phi = q_YZ(1) + q_YZ(2);       % Convert our phi from Body_coord to World_coord.
% YZ_real_dphi = diff(YZ_real_phi, t);      % Get angular velocity in World_coord.
function dxdt = self_balance(t, x, A, B, K)
    u = -K * x;
    dxdt = A * x + B * u; 
end
