function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)
% For 2.3

% Extract angular rates
p = var(10);
q = var(11);
r = var(12);

% Hover thrust
Fc = [0; 0; m*g];

% constant gains
Kp = 0.02;
Kq = 0.02;
Kr = 0.01;

% Rate Feedback control
Gc = [
    -Kp * p
    -Kq * q
    -Kr * r
];

end