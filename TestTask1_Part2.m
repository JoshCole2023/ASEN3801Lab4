clear;close all; clc

% Constants
m = 0.068;
g = 9.81;
f_trim = (m * g) / 4;
motor_forces_hover = [f_trim; f_trim; f_trim; f_trim];

% Initial conditions
var0 = zeros(12, 1); 
tspan = [0 10];

% Parameters
I = diag([5.8e-5, 7.2e-5, 1.0e-4]);
d = 0.060; km = 0.0024; nu = 1e-3; mu = 2e-6;

[t, states] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces_hover), tspan, var0);

% Verify: If trim is correct, states(end, :) should remain near zero.