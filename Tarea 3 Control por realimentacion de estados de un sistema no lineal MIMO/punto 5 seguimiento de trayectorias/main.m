clc, clear, close all

% Seguimiento de trayectorias
l = 0.5;
tau = 20.0;
dt = 0.01;

x1 = [3 3 0]';
[X1,Y1,U1, E1, T] = tracking(x1, l, tau, dt);

x2 = [-5 1 pi]';
[X2,Y2, U2, E2, T] = tracking(x2, l, tau, dt);

x3 = [-1 5 pi/4]';
[X3,Y3,U3, E3, T] = tracking(x3, l, tau, dt);

plot_results(Y1, Y2, Y3, U1, U2, U3, E1, E2, E3, T);