clc
clear all
close all

%% Robot Parameter
robot_parameters = [567;76;524;1244;131;164;327;22;44]/1000;

%% Desired Trajectory  - Circle
th = 0:pi/30:2*pi;
x = 0.5 * cos(th);
y = 0.5 * sin(th);
z = -1.1;

for i = 1:length(x)
    q = IK([x(i),y(i),z],robot_parameters)
    delta_robot(q,robot_parameters)
    J = Jacobian(q,[x(i),y(i),z],robot_parameters)
    pause(0.05)
end
