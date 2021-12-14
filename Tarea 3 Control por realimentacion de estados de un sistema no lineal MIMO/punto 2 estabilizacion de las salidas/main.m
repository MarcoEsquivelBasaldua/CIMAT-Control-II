clc, clear, close all

% Estabilizacion de las salidas
l = 0.5;
t_end = 3.0;
dt = 0.01;

x1 = [3 3 0]';
[X1,Y1,U1,T] = estab_salidas(x1, l, t_end, dt);

x2 = [-5 1 pi]';
[X2,Y2,U2,T] = estab_salidas(x2, l, t_end, dt);

x3 = [-1 5 pi/4]';
[X3,Y3,U3,T] = estab_salidas(x3, l, t_end, dt);

plot_results(Y1, Y2, Y3, U1, U2, U3, T);