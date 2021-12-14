% Compute end effector position once every A matrix simplified
clc, clear
 syms c1 c2 c3 c4 c5 c6 s1 s2 s3 s4 s5 s6 a2 d1 d2 d3 d4
 
 A1 = [c1 0 s1 0;s1 0 -c1 0;0 1 0 d1;0 0 0 1]
 A2 = [c2 -s2 0 a2*c2;s2 c2 0 a2*s2; 0 0 1 d2;0 0 0 1]
 A3 = [c3 0 -s3 0;s3 0 c3 0;0 1 0 d3; 0 0 0 1]
 A4 = [c4 0 s4 0;s4 0 -c4 0;0 1 0 d4;0 0 0 1]
 A5 = [c5 0 -s5 0;s5 0 c5 0;0 1 0 0;0 0 0 1]
 A6 = [c6 -s6 0 0;s6 c6 0 0;0 0 1 0;0 0 0 1]
 
 T_06 = A1*A2*A3*A4*A5*A6;

 p = T_06(1:3,4)