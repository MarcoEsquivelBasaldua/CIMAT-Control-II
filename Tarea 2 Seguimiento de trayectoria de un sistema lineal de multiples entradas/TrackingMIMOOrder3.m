clc, clear, close all,

%System's matrices
A=[1 -1 0;
   1  2 0;
   -1  3 -1]; 
B=[ 0 1;
   -1 0;
   -1 1]; 
C=[ 1 1 -1;
   -2 1  0];

Cont = [B A*B A*A*B];
rnk = rank(Cont);

%obs=[C; C*A; C*A*A]
obs=[C(1,:); C(1,:)*A; C(2,:); C(2,:)*A]

T=obs(1:3,1:3)
rankT=rank(T)

Ae=T*A*inv(T);
Be=T*B;
Ber=[Be(2,:); Be(3,:)],
rank(Ber),

polEr=rank(Ae);

Ke = place(Ae,Be,[-5 -8 -10])

%Initial conditions
x = [1 -1 0]';

%Arrays to save data
tAr=[]; xAr=[]; eAr=[]; yAr=[]; yrAr=[]; uAr = [];

dt=0.01;
for t=0:dt:8

    yr1=2;
    dyr1=0;
    ddyr1=0;
    yr2=sin(2*t);
    dyr2=2*cos(2*t);
    
    rv=[yr1; dyr1; yr2];
    e=T*x-rv;
    rvT=T*A*inv(T)*rv;
    
    %Opcion 1 asignando dinamicas independientes para cada bloque, pero se necesita
    %eliminar la dinamica de cada uno, que son los terminos Ae*e
    v1_aux=-[2 3]*e(1:2); %Polos para los 2 primeros errores
    v2_aux=-[2]*e(3); %Directamente el polo para el tercer error
    u=inv(Ber)*[-Ae(2,:)*e+v1_aux-rvT(2)+ddyr1; -Ae(3,:)*e+v2_aux-rvT(3)+dyr2];
    
    %Opcion 2 asignando la dinamica completa mediante reubicacion de todos los polos
    %u=-Ke*e+inv(Ber)*([-rvT(2)+ddyr1; -rvT(3)+dyr2]);

    y=C*x;   
    x=x+dt*(A*x+B*u);

    
    tAr=[tAr t]; xAr=[xAr x];
    yAr=[yAr y]; yrAr=[yrAr [yr1;yr2]]; eAr=[eAr e]; uAr = [uAr u];
end

figure()
plot(tAr,xAr), legend('x_1','x_2','x_3'),ylabel('State variables(#3) - x')
xlabel('Time (s)')
figure()
plot(tAr,eAr), title('Error variables'), legend('e_1','e_2','e_3'),
figure()
subplot(2,1,1), plot(tAr,yrAr,'--',tAr,yAr), 
title('Outputs and References'), legend('y_1^r','y_2^r','y_1','y_2'),
subplot(2,1,2), plot(tAr,uAr,'--'), title('Control Inputs'),
