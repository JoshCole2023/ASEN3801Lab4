function var_dot = QuadrotorEOM_Linearized(~, var, g, m, I)

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

% Position kinematics
var_dot(1) = u;
var_dot(2) = v;
var_dot(3) = w;

% Attitude kinematics
var_dot(4) = p;
var_dot(5) = q;
var_dot(6) = r;

% Translational dynamics
var_dot(7) = g * theta;
var_dot(8) = -g * phi;
var_dot(9) = 0;

% Rotational dynamics (free motion)
var_dot(10) = 0; % p_dot
var_dot(11) = 0; % q_dot
var_dot(12) = 0; % r_dot

end