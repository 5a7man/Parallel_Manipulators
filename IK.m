function q = IK(p,robot_parameters);
%% Position  of end effector
x = p(1);
y = p(2);
z = p(3);

%% Robot's Parameters
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

%% Joint 1
E1 = 2*L*(y+a);
F1 = 2*z*L;
G1 = x^2 + y^2 + z^2 + a^2 + L^2 + 2*y*a - l^2;

if E1^2+F1^2-G1^2<0
    disp('q1 unacceptable');
else
    t = (-F1-sqrt(E1^2+F1^2-G1^2))/(G1-E1);
    q1 = 2*atand((t));
end

%% Joint 2
E2 = -L*(sqrt(3)*(x+b)+y+c);
F2 = 2*z*L;
G2 = x^2 + y^2 + z^2 + b^2 + c^2 + L^2 + 2*x*b + 2*y*c - l^2;

if E2^2+F2^2-G2^2<0
    disp('q2 unacceptable');
else
    t = (-F2-sqrt(E2^2+F2^2-G2^2))/(G2-E2);
    q2 = 2*atand((t));
end


%% Joint 3
E3 = L*(sqrt(3)*(x-b)-y-c);
F3 = 2*z*L;
G3 = x^2 + y^2 + z^2 + b^2 + c^2 + L^2 - 2*x*b + 2*y*c - l^2;

if E3^2+F3^2-G3^2<0
    disp('q3 unacceptable');
else
    t = (-F3-sqrt(E3^2+F3^2-G3^2))/(G3-E3);
    q3 = 2*atand((t));
end

%% Combining 

q = [q1;q2;q3];
end