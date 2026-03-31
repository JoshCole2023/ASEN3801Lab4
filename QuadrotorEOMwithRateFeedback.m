function var_dot = QuadrotorEOMwithRateFeedback(~, var, g, m, I, d, km, nu, mu)

Ix = I(1,1);
Iy = I(2,2);
Iz = I(3,3);

phi = var(4);
theta = var(5);

u = var(7);
v = var(8);
w = var(9);

p = var(10);
q = var(11);
r = var(12);

var_dot = zeros(12,1);

%% Kinematics
var_dot(1:3) = [u; v; w];
var_dot(4) = p;
var_dot(5) = q;
var_dot(6) = r;

%% Translational (linearized)
var_dot(7) = g * theta;
var_dot(8) = -g * phi;
var_dot(9) = 0;

%% Controller
Kp = 0.02;
Kq = 0.02;
Kr = 0.01;

var_dot(10) = -Kp * p / Ix;
var_dot(11) = -Kq * q / Iy;
var_dot(12) = -Kr * r / Iz;

end