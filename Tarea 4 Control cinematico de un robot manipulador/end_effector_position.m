% Get enf-efector position as function of q vector

clc, clear
syms q1 q2 q3 q4 q5 q6 a2 d1 d2 d3 d4

a = [0, a2, 0, 0, 0, 0];
alpha = [pi/2, 0, -pi/2, pi/2, -pi/2, 0];
d = [d1, d2, d3, d4, 0, 0];
q = [q1, q2, q3, q4, q5, q6];

A1 = compute_dh_matrix(a(1), alpha(1), d(1), q(1));
A2 = compute_dh_matrix(a(2), alpha(2), d(2), q(2));
A3 = compute_dh_matrix(a(3), alpha(3), d(3), q(3));
A4 = compute_dh_matrix(a(4), alpha(4), d(4), q(4));
A5 = compute_dh_matrix(a(5), alpha(5), d(5), q(5));
A6 = compute_dh_matrix(a(6), alpha(6), d(6), q(6));

T_06 = A1*A2*A3*A4*A5*A6;

p = vpa(T_06(1:3,4))