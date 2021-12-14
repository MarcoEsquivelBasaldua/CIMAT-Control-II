clc, clear, close all

% Estabilizacion de las salidas
vi = 1;
t_end = 5.0;
dt = 0.01;

xi = [3 3 0]';
[X1,Y1,U1,T] = estab_ext_dinamica(xi, vi, t_end, dt);

xi = [-5 1 pi]';
[X2,Y2,U2,T] = estab_ext_dinamica(xi, vi, t_end, dt);

xi = [-1 5 pi/4]';
[X3,Y3,U3,T] = estab_ext_dinamica(xi, vi, t_end, dt);

plot_results(Y1, Y2, Y3, U1, U2, U3, T);