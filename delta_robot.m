function delta_robot(q,robot_parameters)
% cla
hold on
view(3) 
grid on
axis([-1 1 -1 1 -1.5 0.3])
%% Joints' Angles
q1 = q(1);
q2 = q(2);
q3 = q(3);

%% End Effector Positon
p = FK(q,robot_parameters)
x = p(1);
y = p(2);
z = p(3);

%% Robot Parameters
s_b = robot_parameters(1);
s_p = robot_parameters(2);
L = robot_parameters(3);
l = robot_parameters(4);
h = robot_parameters(5);
w_b = robot_parameters(6);
u_b = robot_parameters(7);
w_p = robot_parameters(8);
u_p = robot_parameters(9);

%% Base
b1 = [s_b/2,-w_b,0];
b2 = [0,u_b,0];
b3 = [-s_b/2,-w_b,0];
viscircles([0,0],u_b,'color','black');
% patch([b1(1),b2(1),b3(1)],[b1(2),b2(2),b3(2)],[b1(3),b2(3),b3(3)],[1,1,1],'LineWidth', 3);

%% Platform
P1 = [x,y,z]+[0,-u_p,0];
P2 = [x,y,z]+[s_p/2,w_p,0];
P3 = [x,y,z]+[-s_p/2,w_p,0];
patch([P1(1),P2(1),P3(1)],[P1(2),P2(2),P3(2)],[P1(3),P2(3),P3(3)],[0.1,0.1,0.1]);

P1_1 = [-h/2,0,0] + P1;
P1_2 = [h/2,0,0] + P1; 
plot3([P1_1(1), P1_2(1)],[P1_1(2), P1_2(2)],[P1_1(3), P1_2(3)],'b','LineWidth',1)

P2_1 = [-h/2,0,0]*Rz(2*pi/3)*Rz(2*pi/3) + P2;
P2_2 = [h/2,0,0]*Rz(2*pi/3)*Rz(2*pi/3) + P2; 
plot3([P2_1(1), P2_2(1)],[P2_1(2), P2_2(2)],[P2_1(3), P2_2(3)],'b','LineWidth',1)

P3_1 = [-h/2,0,0]*Rz(2*pi/3) + P3;
P3_2 = [h/2,0,0]*Rz(2*pi/3) + P3; 
plot3([P3_1(1), P3_2(1)],[P3_1(2), P3_2(2)],[P3_1(3), P3_2(3)],'b','LineWidth',1)

%% Hip Joints 
B1 = [0,-w_b,0];
B2 = [0.5*sqrt(3)*w_b,w_b/2,0];
B3 = [-0.5*sqrt(3)*w_b,w_b/2,0];
plot3([0,B1(1),B2(1),B3(1)],[0,B1(2),B2(2),B3(2)],[0,B1(3),B2(3),B3(3)],'ro','MarkerSize',2,'LineWidth', 5);

%% Knees Joints
A1 = [0,-w_b-L*cosd(q1),-L*sind(q1)];
A2 = [0.5*sqrt(3)*(w_b+L*cosd(q2)),0.5*(w_b+L*cosd(q2)),-L*sind(q2)];
A3 = [-0.5*sqrt(3)*(w_b+L*cosd(q3)),0.5*(w_b+L*cosd(q3)),-L*sind(q3)];
% plot3([A1(1),A2(1),A3(1)],[A1(2),A2(2),A3(2)],[A1(3),A2(3),A3(3)],'co','MarkerSize',2,'LineWidth', 5);

A1_1 = [-h/2,0,0] + A1;
A1_2 = [h/2,0,0] + A1; 
plot3([A1_1(1), A1_2(1)],[A1_1(2), A1_2(2)],[A1_1(3), A1_2(3)],'b','LineWidth',1)

A2_1 = [-h/2,0,0]*Rz(2*pi/3)*Rz(2*pi/3) + A2;
A2_2 = [h/2,0,0]*Rz(2*pi/3)*Rz(2*pi/3) + A2; 
plot3([A2_1(1), A2_2(1)],[A2_1(2), A2_2(2)],[A2_1(3), A2_2(3)],'b','LineWidth',1)

A3_1 = [-h/2,0,0]*Rz(2*pi/3) + A3;
A3_2 = [h/2,0,0]*Rz(2*pi/3) + A3; 
plot3([A3_1(1), A3_2(1)],[A3_1(2), A3_2(2)],[A3_1(3), A3_2(3)],'b','LineWidth',1)
%% Upper Arms
plot3([B1(1), A1(1)],[B1(2), A1(2)],[B1(3), A1(3)],'-k','LineWidth', 1.5);
plot3([B2(1), A2(1)],[B2(2), A2(2)],[B2(3), A2(3)],'-k','LineWidth', 1.5);
plot3([B3(1), A3(1)],[B3(2), A3(2)],[B3(3), A3(3)],'-k','LineWidth', 1.5);


%% Lower Arms
plot3([A1_1(1), P1_1(1)],[A1_1(2), P1_1(2)],[A1_1(3), P1_1(3)],'-b','LineWidth', 1);
plot3([A1_2(1), P1_2(1)],[A1_2(2), P1_2(2)],[A1_2(3), P1_2(3)],'-b','LineWidth', 1);
plot3([A2_1(1), P2_1(1)],[A2_1(2), P2_1(2)],[A2_1(3), P2_1(3)],'-b','LineWidth', 1);
plot3([A2_2(1), P2_2(1)],[A2_2(2), P2_2(2)],[A2_2(3), P2_2(3)],'-b','LineWidth', 1);
plot3([A3_1(1), P3_1(1)],[A3_1(2), P3_1(2)],[A3_1(3), P3_1(3)],'-b','LineWidth', 1);
plot3([A3_2(1), P3_2(1)],[A3_2(2), P3_2(2)],[A3_2(3), P3_2(3)],'-b','LineWidth', 1);
%% Middle Arm
plot3([0,x],[0,y],[0,z],'-b','LineWidth', 1);

%% End Effector
plot3(x,y,z,'*','color','red')
end


function T = Rz(q)

Sq = sin(q);    Cq = cos(q);

T = [   Cq  -Sq   0 ; 
        Sq   Cq   0 ;  
        0    0    1 ];
end