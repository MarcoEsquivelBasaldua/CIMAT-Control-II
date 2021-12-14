function plot_results(Y1, Y2, Y3, U1, U2, U3, T)

% Salidas del primer caso
Y11 = Y1(1,:);
Y21 = Y1(2,:);

% Salidas del segundo caso
Y12 = Y2(1,:);
Y22 = Y2(2,:);

% Salidas del tercer caso
Y13 = Y3(1,:);
Y23 = Y3(2,:);

% Entradas del primer caso
v1 = U1(1,:);
w1 = U1(2,:);

% Entradas del segundo caso
v2 = U2(1,:);
w2 = U2(2,:);

% Entradas del tercer caso
v3 = U3(1,:);
w3 = U3(2,:);

%figure
subplot(1,3,1)
plot(Y11,Y21)
title('Condicion inicial x1');
xlabel('y1')
ylabel('y2')
grid on

subplot(1,3,2)
plot(Y12,Y22)
title('Condicion inicial x2');
xlabel('y1')
ylabel('y2')
grid on

subplot(1,3,3)
plot(Y13,Y23)
title('Condicion inicial x3');
xlabel('y1')
ylabel('y2')
grid on


figure
subplot(2,2,1)
plot(T, Y11, T, Y12, T,Y13)
title('Salidas y1')
legend('y11','y12','y13')
xlabel('Time')
ylabel('y1')
grid on

subplot(2,2,2)
plot(T, Y21, T, Y22, T,Y23)
title('Salidas y2')
legend('y21','y22','y23')
xlabel('Time')
ylabel('y2')
grid on

subplot(2,2,3)
plot(T, v1, T, v2, T,v3)
title('Entradas v')
legend('v1','v2','v3')
xlabel('Time')
ylabel('v')
grid on

subplot(2,2,4)
plot(T, w1, T, w2, T,w3)
title('Entradas w')
legend('w1','w2','w3')
xlabel('Time')
ylabel('w')
grid on


end