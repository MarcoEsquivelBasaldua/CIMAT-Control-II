%Controllability & Observability
function control_reza(A,B,C,D)
%Checking Controllability Of System
n=size(A,1);
Phi_C=ctrb(A,B);
if rank(Phi_C)==n
    disp('The System Is Controllable')
    flag_C=1;
else
    flag_C=0;
    disp('The System is not Controllable!');
end
%Controllability Indices,Controllability Index & Partial Controllability Matrix
m=size(B,2);
Mu_C=zeros(m,1);count=0;
for i=1:n
    for j=1:m
        count=count+1;
        if count==1
            F=Phi_C(:,1); Mu_C(j)= Mu_C(j)+1;
        else
            if rank([F Phi_C(:,count)]) > rank(F)
                F=[F Phi_C(:,count)]; Mu_C(j)= Mu_C(j)+1;
            end
        end
    end
end
disp('Controllability Indices is:')
disp(Mu_C)
Controllability_Index=max(Mu_C);               % Controllability Index
disp('Controllability_Index is:')
disp(Controllability_Index)
Partial_Controllability_Matrix=B;               % Partial Controllability Matix
for i=1:Controllability_Index-1
    Partial_Controllability_Matrix=[Partial_Controllability_Matrix (A^i)*B];
end
disp('Partial_Controllability_Matrix is:')
disp(Partial_Controllability_Matrix)
%Checking Observability Of System
Phi_O=obsv(A,C);
if rank(Phi_O)==n
    disp('The System Is Observable')
flag_O=1;
else
    flag_O=0;
    disp('The System is not Observable');
end
%Observability Indices,Observability Index & Partial Observability Matrix
l=size(C,1);
Mu_O=zeros(l,1);
count=0;
for i=1:n
    for j=1:l
        count=count+1;
        if count==1
            f=Phi_O';
            F=f(:,1); Mu_O(j)= Mu_O(j)+1;
        else
            if rank([F f(:,count)]) > rank(F)
                F=[F f(:,count)]; Mu_O(j)= Mu_O(j)+1;
            end
        end
    end
end
disp('Obsevability Indices is:')
disp(Mu_O);
Observability_Index=max(Mu_O);               %Observability_Index
disp('Observability_Index is:')
disp(Observability_Index)
Partial_Observibility_Matix=C';              %Partial_Observability_Matrix
for i=1:Observability_Index-1
    Partial_Observibility_Matix=[Partial_Observibility_Matix (A')^i*C'];
end
disp('Partial Observibility Matix is:')
disp(Partial_Observibility_Matix)
%Checking Output Controllability Of System
if flag_C*flag_O==1                      %It means that if the system is minimal then it is Output Controllable.
    disp('The system is Output Controllable');
else
    disp('The system is not Output Controllable');
end
%Checking Functional Controllability Of System
syms s
I=eye(n);
G=C*(inv(s*I-A))*B+D;
r=rank(G);
if r==size(G,1);
    disp('The system Is Functional Output Controllable')
else
    disp('The system Is not Functional Output Controllable');
end
end
%Written By:Seyyed Reza Jafari
%Univercity Of K.N.Tu Iran
%Email:engreza95@gmail.com
%November 2017