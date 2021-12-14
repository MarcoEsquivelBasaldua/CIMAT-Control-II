clc, clear, close all

% Estabilizacion de las salidas
l = 0.5;
tau = 3.0;
dt = 0.01;

x1 = [3 3 0]';
yf = [1 -3]';
[X1,Y1,U1, T] = line_track(x1, yf, l, tau, dt);

x2 = [-5 1 pi]';
[X2,Y2, U2, T] = line_track(x2, yf, l, tau, dt);

x3 = [-1 5 pi/4]';
[X3,Y3,U3,T] = line_track(x3, yf, l, tau, dt);

plot_results(Y1, Y2, Y3, U1, U2, U3, T);