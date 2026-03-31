%% Task 2 Main Script - Linear Equations and Simple Control
clear; 
clc; 
close all;

%% parameters
m = 0.068;
g = 9.81;

Ixx = 5.8e-5;
Iyy = 7.2e-5;
Izz = 1.0e-4;
I = diag([Ixx Iyy Izz]);

d = 0.060;
km = 0.0024;

nu = 0;
mu = 0;

tspan = [0 10];
deg = pi/180;

% hover thrust
f_trim = m*g/4;
motor_trim = [f_trim; f_trim; f_trim; f_trim];

% disturbance cases
cases = {
    [0 0 0 5*deg 0 0 0 0 0 0 0 0]' , 'roll angle'
    [0 0 0 0 5*deg 0 0 0 0 0 0 0]' , 'pitch angle'
    [0 0 0 0 0 5*deg 0 0 0 0 0 0]' , 'yaw angle'
    [0 0 0 0 0 0 0 0 0 0.1 0 0]' , 'roll rate (p)'
    [0 0 0 0 0 0 0 0 0 0 0.1 0]' , 'pitch rate (q)'
    [0 0 0 0 0 0 0 0 0 0 0 0.1]' , 'yaw rate (r)'
};

%% Task 2.1 — Nonlinear Model
for i = 1:length(cases)
    
    x0 = cases{i,1};
    label = cases{i,2};


    [t,x] = ode45(@(t,var) QuadrotorEOMFunc(t,var,g,m,I,d,km,nu,mu,motor_trim), tspan, x0);

    figure
    subplot(2,1,1)
    plot(t,x(:,4),'LineWidth',1.5); hold on
    plot(t,x(:,5),'LineWidth',1.5)
    plot(t,x(:,6),'LineWidth',1.5)
    legend('\phi','\theta','\psi')
    title(['Task 2.1 Nonlinear - ' label])
    ylabel('Angle (rad)')
    grid on

    subplot(2,1,2)
    plot(t,x(:,10),'LineWidth',1.5); hold on
    plot(t,x(:,11),'LineWidth',1.5)
    plot(t,x(:,12),'LineWidth',1.5)
    legend('p','q','r')
    xlabel('Time (s)')
    ylabel('Angular Rates (rad/s)')
    grid on
end

%% Task 2.2 — Linearized Model
for i = 1:length(cases)
    
    x0 = cases{i,1};

    [t,x] = ode45(@(t,var) QuadrotorEOM_Linearized(t,var,g,m,I), tspan, x0);

    figure
    subplot(2,1,1)
    plot(t,x(:,4),'LineWidth',1.5); hold on
    plot(t,x(:,5),'LineWidth',1.5)
    plot(t,x(:,6),'LineWidth',1.5)
    legend('\phi','\theta','\psi')
    title(['Task 2.2 Linearized - ' label])
    ylabel('Angle (rad)')
    grid on

    subplot(2,1,2)
    plot(t,x(:,10),'LineWidth',1.5); hold on
    plot(t,x(:,11),'LineWidth',1.5)
    plot(t,x(:,12),'LineWidth',1.5)
    legend('p','q','r')
    xlabel('Time (s)')
    ylabel('Angular Rates (rad/s)')
    grid on
end

%% Task 2.5 — Controlled vs Uncontrolled Model
for i = 1:length(cases)
    x0 = cases{i,1};

    % uncontrolled
    [t1,x1] = ode45(@(t,var) QuadrotorEOMFunc(t,var,g,m,I,d,km,nu,mu,motor_trim), tspan, x0);

    % controlled
    [t2,x2] = ode45(@(t,var) QuadrotorEOMwithRateFeedback(t,var,g,m,I,d,km,nu,mu), tspan, x0);

    figure

    subplot(2,1,1)
    plot(t1,x1(:,4),'b--','LineWidth',1.5); hold on
    plot(t2,x2(:,4),'r','LineWidth',1.5)
    legend('uncontrolled','controlled')
    title(['Task 2.5 - ' label])
    ylabel('\phi (rad)')
    grid on

    subplot(2,1,2)
    plot(t2,x2(:,10),'r','LineWidth',1.5)
    xlabel('Time (s)')
    ylabel('p (rad/s)')
    grid on
end