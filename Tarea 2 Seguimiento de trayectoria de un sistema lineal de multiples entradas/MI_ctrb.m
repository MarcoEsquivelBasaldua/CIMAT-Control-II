function [T,Ad,Bd,ni]=mi_ctrb(A,B)

%MIMO Controllable Canonical Form
% FUNCTION [T,Ad,Bd,ni]=MI_CTRB(A,B)
% For a multi-input control system with state matrix A and input matrix B,
% the function would compute the transformation matrix T, such that
% the matrices Ad=TAT^(-1) and Bd=TB would be in comtrollable canonical form.
%
% Example :
% a = [-0.6018   -4.855   0.3625   -1.379    3.005   -1.506    1.104
%     4.955  -0.1506   -7.368     -9.9    3.513  -0.2289   -2.583
%     -0.1107    7.225  -0.6343    2.096    3.523   -2.823    1.436
%     0.9662    10.07   -1.246   -0.539     1.87   -1.714  -0.6016
%     -2.834   -3.387   -3.467   -2.104  -0.4699   -2.716    1.563
%     1.419   0.2755    3.101     1.35    2.571  -0.3424   0.7486
%     -1.606    2.685     -1.2   0.3355   -1.371  -0.3118  -0.3507];
% 
% 
% b = [-1.508   0.05876    -1.039
%     0.1867    0.6734         0
%     -2.857   -0.9217   -0.3253
%     0.2794   -0.6649    -1.118
%     -0.2849   -0.2516   -0.1021
%     1.039    0.5565  -0.08114
%     0         0         0];
%
% [T,Ad,Bd]=mi_ctrb(a,b);
% gives the output Ad and Bd as
% Ad =
%          0    1.0000         0         0         0         0         0
%          0         0    1.0000         0         0         0         0
%     8.4648 -122.9418   -0.3338    0.9968   -2.8377   -0.0650    1.7982
%          0         0         0         0    1.0000         0         0
%   139.1784 -108.8978         0   11.7956   -2.3174    1.3238    5.9512
%          0         0         0         0         0         0    1.0000
%  -279.4737 -228.2555         0  -32.6091  -24.1423    1.9887   -0.4375
%  
%  Bd =
%          0         0         0
%          0         0         0
%     1.0000    0.7005    0.5448
%          0         0         0
%          0    1.0000         0
%          0         0         0
%          0         0    1.0000
error(nargchk(2,2,nargin))
error(nargoutchk(1,4,nargout))

if ~isreal(A) || ~isreal(B)
    error('Matrices A and B are supposed to have real elements');
end

[n1 n2]=size(A);
[n m]=size(B);

if (n1 ~= n2)
    error('A should be a square matrix');
end
if (n1 ~= n)
    error('Matrices A and B should have same number of rows');
end

Ct=[];
Q=eye(n);
is_ctrb=(1<0); % Controllability Not yet confirmed. Assumed not controllable for now

for i=0:(n-1)

    Ct=[Ct Q*B];
    is_ctrb=(rank(Ct)==n);
    if is_ctrb
        break;
    else
        Q=Q*A;
    end
end

if ~is_ctrb
    error('System does not seem to be controllable. Cannot deduce transformation matrix');
end

no_col=size(Ct,2);
ni(m)=0;
jvec=1:m;
for i=0:((no_col/m)-1)
    for j=jvec
        if rank(Ct(:,1:(m*i+j-1))) < rank(Ct(:,1:(m*i+j)))
            ni(j)=ni(j)+1;
        else
            jvec=setdiff(jvec,j);
        end
    end
end

Cmat=[];
for i=1:m
    cols=(0:(ni(i)-1))*m+i;
    Cmat=[Cmat Ct(:,cols)];
end
DD=inv(Cmat);
Qmat=zeros(m,n);
T=[];

for i=1:m
    if ni(i) > 0
        Qmat(i,:)=DD(sum(ni(1:i)),:);
        Q=eye(n);
        for j=0:(ni(i)-1)
            T=[T;Qmat(i,:)*Q];
            Q=Q*A;
        end
    end
end

if nargout > 1
    Ad=clean(T*A*inv(T));
    Bd=clean(T*B);
end

function y=clean(y)
y(find(abs(y)<1e-10*max(max(abs(y)))))=0;
return
