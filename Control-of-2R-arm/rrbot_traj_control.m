clear; close; clc;

%ROS Setup
rosinit;

j1_effort = rospublisher('/rrbot/joint1_effort_controller/command');
j2_effort = rospublisher('/rrbot/joint2_effort_controller/command');
JointStates = rossubscriber('/rrbot/joint_states');

tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j2_effort);

tau1.Data = 0;
tau2.Data = 0;

send(j1_effort,tau1);
send(j2_effort,tau2);

client = rossvcclient('/gazebo/set_model_configuration');

req = rosmessage(client);
req.ModelName = 'rrbot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1','joint2'};
req.JointPositions = [deg2rad(200), deg2rad(125)];

resp = call(client,req,'Timeout',3);

tic;

t = 0;

i = 1;

% Input data U1 U2 
Tau1_Data = [0];
Tau2_Data = [0];

% Position and Velocity data
Theta1_Data = [deg2rad(200)];
Theta2_Data = [deg2rad(125)];
Theta1_dot_Data = [receive(JointStates).Velocity(1)];
Theta2_dot_Data = [receive(JointStates).Velocity(2)];


T = [];


while(t < 10)
    t = toc;
    
    % read the joint states
    jointData = receive(JointStates);   %inspect the "jointData" variable in MATLAB to get familiar with its structure
    theta1 = jointData.Position(1);
    theta2 = jointData.Position(2);
    theta1_dot = jointData.Velocity(1);
    theta2_dot = jointData.Velocity(2);
    
    

    % design your state feedback controller in the following
    if abs(theta1) > 2*pi
        theta1 = mod(theta1 ,2*pi);
    end
    
    if abs(theta2) > 2*pi
        theta2 = mod( theta2 ,2*pi);
    end
    

    Kp = 10; Kd = 5;

    q1_desired = (pi*t^3)/500 - (3*pi*t^2)/100 + pi;
    q2_desired = (pi*t^3)/1000 - (3*pi*t^2)/200 + pi/2;
    q1_dot_desired = (3*pi*t^2)/500 - (3*pi*t)/50;
    q2_dot_desired = (3*pi*t^2)/1000 - (3*pi*t)/100;
    q1_ddot_desired = (3*pi*t)/250 - (3*pi)/50;
    q2_ddot_desired = (3*pi*t)/500 - (3*pi)/100;

        
    u1 = ((9*cos(theta2))/10 + 1573/1000)*(q1_ddot_desired + Kd*(q1_dot_desired - theta1_dot) + Kp*(q1_desired - theta1)) - (28449*sin(theta1))/2000 - (8829*sin(theta1 + theta2))/2000 + ((9*cos(theta2))/20 + 573/2000)*(q2_ddot_desired + Kd*(q2_dot_desired - theta2_dot) + Kp*(q2_desired - theta2)) - (theta2_dot*((9*theta1_dot*sin(theta2))/5 + (9*theta2_dot*sin(theta2))/10))/2;
    u2 = (573*q2_ddot_desired)/2000 - (8829*sin(theta1 + theta2))/2000 + (9*theta1_dot^2*sin(theta2))/20 + (573*Kd*(q2_dot_desired - theta2_dot))/2000 + (573*Kp*(q2_desired - theta2))/2000 + ((9*cos(theta2))/20 + 573/2000)*(q1_ddot_desired + Kd*(q1_dot_desired - theta1_dot) + Kp*(q1_desired - theta1));

    tau1.Data = u1;
    tau2.Data = u2;

    % you can sample data here to plot at the end
    Tau1_Data(i) = tau1.Data;
    Tau2_Data(i) = tau2.Data;
    
    Theta1_Data(i) = theta1;
    Theta2_Data(i) = theta2;
    Theta1_dot_Data(i) = theta1_dot;
    Theta2_dot_Data(i) = theta2_dot;

    send(j1_effort,tau1);
    send(j2_effort,tau2);
    
    T(i) = t;
    i = i+1;
end

tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);

% disconnect from roscore 
rosshutdown;

figure(1)
subplot(2,2,1)
plot(T,Theta1_Data,'-')
hold on
q1_desired = (pi*T.^3)/500 - (3*pi*T.^2)/100 + pi;
plot(T, q1_desired, '-r'); 
title('theta1 RRBot');
xlabel('Time -->');
ylabel('theta1 (rad)');
legend('theta1', 'theta1 desired');


subplot(2,2,2)
plot(T,Theta2_Data,'-')
hold on
theta2_desired = (pi*T.^3)/1000 - (3*pi*T.^2)/200 + pi/2;
plot(T, theta2_desired); 
title('theta2 RRBot');
xlabel('Time -->');
ylabel('theta2 (rad)');
legend('theta2', 'theta2 desired');

subplot(2,2,3)
plot(T,Theta1_dot_Data,'-')
hold on
theta1_dot_desired = (3*pi*T.^2)/500 - (3*pi*T)/50;
plot(T, theta1_dot_desired); 
title('theta1(dot) RRBot');
xlabel('Time -->');
ylabel('theta1 dot (rad/s)');
legend('theta1 dot', 'theta1 dot desired');

subplot(2,2,4)
plot(T,Theta2_dot_Data,'-')
hold on
theta2_ddot_desired = (3*pi*T.^2)/1000 - (3*pi*T)/100;
plot(T, theta2_ddot_desired); 
title('theta2(dot) RRBot');
xlabel('Time -->');
ylabel('theta2 dot (rad/s)');
legend('theta2 dot', 'theta2 dot desired');

figure(2)
subplot(2,1,1)
plot(T,Tau1_Data,'-')
title('Tau1 vs t');
xlabel('Time -->');
ylabel('Tau1 (N/m)');
xlim([0,10]);

subplot(2,1,2)
plot(T,Tau2_Data,'-')
title('Tau2 vs t');
xlabel('Time -->');
ylabel('Tau2 (N/m)');
xlim([0,10]);