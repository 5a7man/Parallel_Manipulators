function p = FK(q,robot_parameters)
%% Joints' Angles 
q1 = q(1);
q2 = q(2);
q3 = q(3);

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

%% Solving for end effector's position 
    function eqn = F_K(position)
        x = position(1);
        y = position(2);
        z = position(3);
        eqn(1) = 2*z*L*sind(q1) + x*x + y*y + z*z - l*l + L*L + a*a + 2*y*a + 2*L*(y+a)*cosd(q1);
        eqn(2) = 2*z*L*sind(q2) + x*x + y*y + z*z - l*l + L*L + b*b + c*c + 2*x*b + 2*y*c - L*(sqrt(3)*(x+b)+y+c)*cosd(q2);
        eqn(3) = 2*z*L*sind(q3) + x*x + y*y + z*z - l*l + L*L + b*b + c*c - 2*x*b + 2*y*c + L*(sqrt(3)*(x-b)-y-c)*cosd(q3);
    end

fun = @F_K;
options = optimset('Display','off');
p = fsolve(fun,[0,0,-500],options);
p  = p'; 

end