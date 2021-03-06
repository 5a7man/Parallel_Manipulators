function J= Jacobian(current_q,current_p,robot_parameters)
%% End effector position
x = current_p(1);
y = current_p(2);
z = current_p(3);

%% Joints' Angles
q1 = current_q(1);
q2 = current_q(2);
q3 = current_q(3);


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

a = w_b - u_p;
b =  0.5*s_p - 0.5*sqrt(3)*w_b;
c =  w_p - 0.5*w_b;

%% Getting del_p
a11 = x;
a12 = y+a + L*cosd(q1);
a13 = z + L*sind(q1);

a21 = 2*(x+b) - sqrt(3)*L*cosd(q2);
a22 = 2*(y + c) - L*cosd(q2);
a23 = 2*(z + L*sind(q2));

a31 = 2*(x - b) + sqrt(3)*L*cosd(q3);
a32 = 2*(y + c) - L*cosd(q3);
a33 = 2*(z + L*sind(q3));

A = [a11,a12,a13;...
    a21,a22,a23;...
    a31,a32,a33];

b11 = L*((y + a)*sind(q1) - z*cosd(q1));
b22 = -L*((sqrt(3)*(x + b) + y + c)*sind(q2) + 2*z*cosd(q2));
b33 = L*((sqrt(3)*(x-b) - y - c)*sind(q3) - 2*z*cosd(q3));
		
B = [b11,0,0;...
    0,b22,0;...
    0,0,b33];

J= inv(A)*B;
end
