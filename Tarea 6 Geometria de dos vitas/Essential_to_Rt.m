function [R1, t1, R2, t2] = Essential_to_Rt(E)

[U,S,V] = svd(E);

W = [0 -1 0; 1 0 0;0 0 1];
Z = [0 1 0;-1 0 0;0 0 0];

% First solution
t1_skew = U*W*S*U';
%t1_skew = U*Z*U';
t1 = [t1_skew(3,2); t1_skew(1,3); t1_skew(2,1)];
R1 = U*W'*V';

t1 = -R1*t1;

% Second solution
W = W';
Z = Z';
t2_skew = U*W*S*U';
%t2_skew = U*Z*U';
t2 = [t2_skew(3,2); t2_skew(1,3); t2_skew(2,1)];
R2 = U*W'*V';

t2 = -R2*t2;
end