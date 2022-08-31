function dz = myode_RRbot(t,z)
g = 9.81; 

%% Define mass (kg)
m1 = 1;
m2 = m1;

%% Define Moment of Inertia (kg . m^2 )
I1 = 0.084;
I2 = I1;

%% Length of links (m)
l1 = 1;
l2 = l1;

%% Position of COM of link
r1 = 0.45;
r2 = r1;

%% ODE Function
dz = zeros(4,1);
z = num2cell(z);
[theta1, theta2, theta1_dot, theta2_dot] = deal(z{:});

if abs(theta1)>2*pi
    theta1 = mod(theta1, 2*pi); % Warp the angle to [0,2pi]
end

if abs(theta2)>2*pi
    theta2 = mod(theta2, 2*pi); % Warp the angle to [0,2pi]
end

% Tune PD gains 
kp = 10; kd = 1; 

q1_desired = (pi*t^3)/500 - (3*pi*t^2)/100 + pi;
q2_desired = (pi*t^3)/1000 - (3*pi*t^2)/200 + pi/2;
q1_dot_desired = (3*pi*t^2)/500 - (3*pi*t)/50;
q2_dot_desired = (3*pi*t^2)/1000 - (3*pi*t)/100;
q1_ddot_desired = (3*pi*t)/250 - (3*pi)/50;
q2_ddot_desired = (3*pi*t)/500 - (3*pi)/100;


u1 = ((9*cos(theta2))/10 + 1573/1000)*(q1_ddot_desired + kd*(q1_dot_desired - theta1_dot) + kp*(q1_desired - theta1)) - (28449*sin(theta1))/2000 - (8829*sin(theta1 + theta2))/2000 + ((9*cos(theta2))/20 + 573/2000)*(q2_ddot_desired + kd*(q2_dot_desired - theta2_dot) + kp*(q2_desired - theta2)) - (theta2_dot*((9*theta1_dot*sin(theta2))/5 + (9*theta2_dot*sin(theta2))/10))/2;
u2 = (573*q2_ddot_desired)/2000 - (8829*sin(theta1 + theta2))/2000 + (9*theta1_dot^2*sin(theta2))/20 + (573*kd*(q2_dot_desired - theta2_dot))/2000 + (573*kp*(q2_desired - theta2))/2000 + ((9*cos(theta2))/20 + 573/2000)*(q1_ddot_desired + kd*(q1_dot_desired - theta1_dot) + kp*(q1_desired - theta1));

dz(1) = theta1_dot;
dz(2) = theta2_dot;
dz(3) = (I2*u1 - I2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*r2^3*theta1_dot^2*sin(theta2) + l1*m2^2*r2^3*theta2_dot^2*sin(theta2) + g*l1*m2^2*r2^2*sin(theta1) + I2*g*l1*m2*sin(theta1) + I2*g*m1*r1*sin(theta1) - l1*m2*r2*u2*cos(theta2) + 2*l1*m2^2*r2^3*theta1_dot*theta2_dot*sin(theta2) + l1^2*m2^2*r2^2*theta1_dot^2*cos(theta2)*sin(theta2) - g*l1*m2^2*r2^2*sin(theta1 + theta2)*cos(theta2) + I2*l1*m2*r2*theta1_dot^2*sin(theta2) + I2*l1*m2*r2*theta2_dot^2*sin(theta2) + g*m1*m2*r1*r2^2*sin(theta1) + 2*I2*l1*m2*r2*theta1_dot*theta2_dot*sin(theta2))/(- l1^2*m2^2*r2^2*cos(theta2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dz(4) = -(I2*u1 - I1*u2 - I2*u2 - l1^2*m2*u2 - m1*r1^2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*r2^3*theta1_dot^2*sin(theta2) + l1^3*m2^2*r2*theta1_dot^2*sin(theta2) + l1*m2^2*r2^3*theta2_dot^2*sin(theta2) - g*l1^2*m2^2*r2*sin(theta1 + theta2) - I1*g*m2*r2*sin(theta1 + theta2) + g*l1*m2^2*r2^2*sin(theta1) + I2*g*l1*m2*sin(theta1) + I2*g*m1*r1*sin(theta1) + l1*m2*r2*u1*cos(theta2) - 2*l1*m2*r2*u2*cos(theta2) + 2*l1*m2^2*r2^3*theta1_dot*theta2_dot*sin(theta2) + 2*l1^2*m2^2*r2^2*theta1_dot^2*cos(theta2)*sin(theta2) + l1^2*m2^2*r2^2*theta2_dot^2*cos(theta2)*sin(theta2) - g*l1*m2^2*r2^2*sin(theta1 + theta2)*cos(theta2) + g*l1^2*m2^2*r2*cos(theta2)*sin(theta1) - g*m1*m2*r1^2*r2*sin(theta1 + theta2) + I1*l1*m2*r2*theta1_dot^2*sin(theta2) + I2*l1*m2*r2*theta1_dot^2*sin(theta2) + I2*l1*m2*r2*theta2_dot^2*sin(theta2) + g*m1*m2*r1*r2^2*sin(theta1) + 2*l1^2*m2^2*r2^2*theta1_dot*theta2_dot*cos(theta2)*sin(theta2) + l1*m1*m2*r1^2*r2*theta1_dot^2*sin(theta2) + 2*I2*l1*m2*r2*theta1_dot*theta2_dot*sin(theta2) + g*l1*m1*m2*r1*r2*cos(theta2)*sin(theta1))/(- l1^2*m2^2*r2^2*cos(theta2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);

end


