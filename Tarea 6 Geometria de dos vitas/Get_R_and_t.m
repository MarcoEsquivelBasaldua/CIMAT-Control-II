% Define R y t
theta = pi/6;
R = [cos(theta) 0 sin(theta);0 1 0; -sin(theta) 0 cos(theta)]
t = [1; 0; 4]


% Calcular homografia y obtener R y t
%se propone adicinal n y zeta
n = [1;0;0];
zeta_ = 0.5;

H = R - zeta_*t*n'

[R1_h,t1_h,n1, R2_h,t2_h,n2, zeta] = homog_to_Rt(H)


% Calcular matriz essencial y obtener R y t
t_skew = [0 -t(3) t(2);t(3) 0 -t(1);-t(2) t(1) 0];
E = R*t_skew

[R1_e, t1_e, R2_e, t2_e] = Essential_to_Rt(E)