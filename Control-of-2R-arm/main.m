% Derive equations of motion or 2-R robot
syms theta1 theta2 theta1_dot theta2_dot theta1_ddot theta2_ddot;
syms r1 l1 r2 l2;
syms m1 m2 I1 I2;
syms g;
syms u1 u2;
syms a0 a1 a2 a3 t;

%% 1. Calculate K1
x1 = r1 * sin(theta1);
y1 = r1 * cos(theta1);

x1_dot = r1 * cos(theta1) * theta1_dot;
y1_dot = (-1) * r1 * sin(theta1) * theta1_dot;

K1_lin = (1/2) * (m1) * (x1_dot^2 + y1_dot^2);
K1_ang = (1/2) * (I1) * (theta1_dot)^2;
K1 = K1_lin + K1_ang;

%% 2. Calculate K2
x2 = l1 * sin(theta1) + r2 * sin(theta1 + theta2);
y2 = l1 * cos(theta1) + r2 * cos(theta1 + theta2);

x2_dot = (l1 * cos(theta1) * theta1_dot) + ( r2 * cos(theta1 + theta2) * (theta1_dot + theta2_dot) );
y2_dot = (-1) * ( (l1 * sin(theta1) * theta1_dot) + ( r2 * sin(theta1 + theta2) * (theta1_dot + theta2_dot) ) );

K2_lin = (1/2) * (m2) * (x2_dot^2 + y2_dot^2);
K2_ang = (1/2) * (I2) * (theta1_dot + theta2_dot)^2;
K2 = K2_lin + K2_ang;


%% 3. Calculate P1
P1 = m1 * g * r1 * cos(theta1);

%% 4. Calculate P2
P2 = m2 * g * (l1 * cos(theta1)  +  r2 * cos(theta1 + theta2) );


%% 5. Calculate Lagrange Equation L = K - P
L = (K1 + K2) - (P1 + P2);
L = simplify(L);

%% 6. State space representation qi (i = 1,2)
qf_dot = [theta1; theta2];
q_dot = [theta1_dot; theta2_dot];
q_ddot = [theta1_ddot; theta2_ddot];

%% 7. To obtain dL/dq and d/dt (dL/dq_dot)
L11 = jacobian(L,theta1); % L11 = partial differntiation of dL/d(theta1)
L12 = jacobian(L, theta1_dot); % L12 = partial differntiation of dL/d(theta1_dot)

L21 = jacobian(L, theta2);  % L21 = partial differntiation of dL/d(theta2)
L22 = jacobian(L, theta2_dot); % L22 = partial differntiation of dL/d(theta2_dot)

diff_L12 = jacobian(L12, [qf_dot; q_dot]) * [q_dot; q_ddot] ;  % time derivative of dL/d(theta1_dot)
diff_L22 = jacobian(L22, [qf_dot; q_dot]) * [q_dot; q_ddot] ;  % time derivative of dL/d(theta2_dot)


%% 8.  d/dt (dL/dq_dot) - dL/dq = u = 0
eq1 = diff_L12 - L11 - u1;
eq2 = diff_L22 - L21 - u2;

%% 9. Solve for theta1_ddot and theta2_ddot
sol = solve([eq1 == 0, eq2 == 0], [theta1_ddot, theta2_ddot] );
simplify(sol.theta1_ddot);
simplify(sol.theta2_ddot);

%% 10.State Space Representation Z
Z(1) = theta1;
Z(2) = theta2;
Z(3) = theta1_dot;
Z(4) = theta2_dot;

Z_dot(1) = theta1_dot;
Z_dot(2) = theta2_dot;
Z_dot(3) = sol.theta1_ddot;
Z_dot(4) = sol.theta2_ddot;

U = [u1;u2];

%% 11. Generate a cubic polynomial trajectory
q_t = a0 + a1*t + a2*(t^2) + a3*(t^3);

% Initial and Final time
t0 = 0;
tf = 10; 

% Initial and Final q and q_dot of joint 1
q0 = deg2rad(180); 
qf = deg2rad(0); 
q0_dot = deg2rad(0); 
qf_dot = deg2rad(0); 

% Joint 1 Trajectory
joint1_T = generate_trajectory(q0, qf, q0_dot, qf_dot, t0, tf);


% Initial and Final q and q_dot of joint 2
q0 = deg2rad(90); 
qf = deg2rad(0); 
q0_dot = deg2rad(0); 
qf_dot = deg2rad(0);

% Joint 2 Trajectory
joint2_T = generate_trajectory(q0, qf, q0_dot, qf_dot, t0, tf);


% To solve for a0, a1, a2, a3, we have q0 q0_dot qf qf_dot
% Joint 1 trajectories
q1_t = subs(q_t, [a0 a1 a2 a3], joint1_T');
q1_dot_t = diff(q1_t);
q1_ddot_t = diff(q1_dot_t);

% Joint 2 trajectories
q2_t = subs(q_t, [a0 a1 a2 a3], joint2_T');
q2_dot_t = diff(q2_t);
q2_ddot_t = diff(q2_dot_t);

%% 12. Convert to Manipulator Form:  M(q)q_ddot + C(q,q_dot)q_dot + g(q) = tau 

% Solve for control inputs U = [u1 u2]
dynamicsEquation = solve([eq1==0, eq2==0],U);

tau(1) = simplify(dynamicsEquation.u1);
tau(2) = simplify(dynamicsEquation.u2);

% Substitute values 
tau = subs(tau, [r1 r2 l1 l2 m1 m2 I1 I2 g], [0.45 0.45 1 1 1 1 0.084 0.084 9.81] );


% Calculate g(q) by substituting q_dot = 0 
G1 = subs(tau(1), [theta1_dot, theta2_dot, theta1_ddot, theta2_ddot], [0 0 0 0]);
G2 = subs(tau(2), [theta1_dot, theta2_dot, theta1_ddot, theta2_ddot], [0 0 0 0]);
G = [G1;G2]

% Calculate C(q,q_dot) by substituting q_ddot = 0 
C1 = (subs(tau(1)-G1, [theta1_ddot theta2_ddot], [0 0]));
C2 = (subs(tau(2)-G2, [theta1_ddot theta2_ddot], [0 0]));
C = [C1;C2]

% Calculate M(q) by tau - g(q) - C(q,q_dot) 
M1 = (tau(1) - G1 - C1 );
M2 = (tau(2) - G2 - C2 );

m11 = subs(M1, theta2_ddot, 0)/theta1_ddot;
m12 = subs(M1, theta1_ddot, 0)/theta2_ddot;
m21 = simplify(subs(M2, theta2_ddot, 0)/theta1_ddot);
m22 = subs(M2, theta1_ddot, 0)/theta2_ddot;

M = [m11 m12; 
     m21 m22]

% Final Manipulator Form:  M(q)q_ddot + C(q,q_dot)q_dot + g(q) = tau
tau = M*q_ddot + C + g

%% 13. Derive the symbolic feedback linearization of the robot
syms Kp Kd; 
syms q1_desired q2_desired q1_dot_desired q2_dot_desired q1_ddot_desired q2_ddot_desired;
syms v1 v2; 

% Virtual Control Inputs (replace u = q_ddot with v)
V = [v1;v2];

q_desired = [q1_desired; q2_desired];
q_dot_desired = [q1_dot_desired; q2_dot_desired];
q_ddot_desired = [q1_ddot_desired; q2_ddot_desired];

% PD Control (State feedback control with feedforward term)
V = -Kp*[ qf_dot - q_desired ] - Kd*[ q_dot - q_dot_desired ] + q_ddot_desired
tau = M*V + C + G;


%% 14. Simulate system for bounded time t = 0 to t = 10 with initial condition for each qi
[t,y] = ode45( @myode_RRbot,[0,10],[deg2rad(200); deg2rad(125); 0; 0] );

%% Plot 

% Plot q vs t
figure(1);
subplot(2,2,1)
plot(t,y(:,1),'-');
hold on
q1_desired = ((pi*t.^3)/500 - (3*pi*t.^2)/100 + pi);
plot(t, q1_desired, '-r'); 
title('theta1 RRBot');
xlabel('Time t -->');
ylabel('theta1 (rad)');
legend('theta1', 'theta1 desired');

subplot(2,2,2)
plot(t,y(:,2),'-');
hold on
q2_desired = ((pi*t.^3)/1000 - (3*pi*t.^2)/200 + pi/2);
plot(t, q2_desired, '-r');
title('theta2 RRBot');
xlabel('Time t -->');
ylabel('theta2 (rad)');
legend('theta2', 'theta2 desired');

% Plot q_dot vs t
subplot(2,2,3)
plot(t,y(:,3),'-');
hold on
q1_dot_desired = ((3*pi*t.^2)/500 - (3*pi*t)/50);
plot(t, q1_dot_desired, '-r')
title('theta1(dot) RRBot');
xlabel('Time t -->');
ylabel('theta1 dot (rad/s)');
legend('theta1 dot', 'theta1 dot desired');

subplot(2,2,4)
plot(t,y(:,4),'-');
hold on
q2_dot_desired = ((3*pi*t.^2)/1000 - (3*pi*t)/100);
plot(t, q2_dot_desired, '-r')
title('theta2(dot) RRBot');
xlabel('Time t -->');
ylabel('theta2 dot (rad/s)');
legend('theta2 dot','theta2 dot desired');

% q = theta1; theta2; theta1_dot; theta2_dot over a period of 10s
theta1 = y(:,1);
theta2 = y(:,2);
theta1_dot = y(:,3);
theta2_dot = y(:,4);

% q_ddot_desired over a period of 10s
q1_ddot_desired = (3*pi*t)/250 - (3*pi)/50;
q2_ddot_desired = (3*pi*t)/500 - (3*pi)/100;

Kp = 10; Kd = 1;

% Virtual Control Input over a period of 10 seconds 
v1_t = -Kp*[ theta1 - q1_desired] - Kd*[ theta1_dot - q1_dot_desired] + q1_ddot_desired;
v2_t = -Kp*[ theta2 - q2_desired] - Kd*[ theta2_dot - q2_dot_desired] + q2_ddot_desired;


% q_t
theta1_t = theta1;
theta2_t = theta2;
theta1_dot_t = theta1_dot;
theta2_dot_t = theta2_dot;

% Initiate an empty matrix and keep adding values to it
tau_1 = [];
tau_2 = [];

% Calculate for each time t, tau1 and tau2
for i = 1:length(v1_t)

    theta1 = theta1_t(i);
    theta2 = theta2_t(i);
    theta1_dot = theta1_dot_t(i);
    theta2_dot = theta2_dot_t(i);
    
    v1 = v1_t(i);
    v2 = v2_t(i);
    
    tau_1_t = v1*((9*cos(theta2))/10 + 1573/1000) - (28449*sin(theta1))/2000 - (8829*sin(theta1 + theta2))/2000 + v2*((9*cos(theta2))/20 + 573/2000) - (theta2_dot*((9*theta1_dot*sin(theta2))/5 + (9*theta2_dot*sin(theta2))/10))/2;
    tau_2_t = (9*sin(theta2)*theta1_dot^2)/20 + (573*v2)/2000 - (8829*sin(theta1 + theta2))/2000 + v1*((9*cos(theta2))/20 + 573/2000);

    % Keep adding tau1 tau2
    tau_1 = [tau_1; tau_1_t];
    tau_2 = [tau_2; tau_2_t];

end

% Plot U vs t
figure(2);
subplot(2,1,1);
plot(t, tau_1,'-')
title('Tau1 vs t');
xlabel('Time -->');
ylabel('Tau1 (N/m)');

subplot(2,1,2)
plot(t, tau_2,'-')
title('Tau2 vs t');
xlabel('Time -->');
ylabel('Tau2 (N/m)');

