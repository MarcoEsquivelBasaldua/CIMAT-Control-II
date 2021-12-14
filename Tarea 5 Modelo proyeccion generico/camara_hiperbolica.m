%%
%% Camara planar
%%
clear, clc, close all
% Parametros de la camara
x_pixels = 768;
y_pixels = 576;

x_m = 6.467e-3;
y_m = 4.831e-3;

f = 6.14495e-3;

m_x = x_pixels/x_m;
m_y = y_pixels/y_m;

a_x = f*m_x;
a_y = f*m_y;

p_x = x_pixels/2;
p_y = y_pixels/2;

d = 0.1;
p = 0.01;

xi = d/sqrt(d^2 +4*p^2);
psi = (d+2*p)/sqrt(d^2 +4*p^2);

K = [a_x 0 p_x;0 a_y p_y;0 0 1]*[psi-xi, 0, 0; 0, psi-xi, 0; 0 0 1];

% Ubicacion de las esquinas del rectangulo
L = 0.5;
W = 1;
l = L/2;
w = W/2;
rect_center = [5,1,1]';

x_rect = zeros(4,1) + rect_center(1);
y_rect = [-l l l -l]' + rect_center(2);
z_rect = [-w -w w w]' + rect_center(3);


labels = cellstr(num2str([1:4]'));
figure()
plot3(x_rect, y_rect, z_rect, '*')
text(x_rect, y_rect, z_rect,labels)
grid on
xlabel('x')
ylabel('y')
zlabel('z')
axis([0 max(x_rect)+1 min(y_rect)-1 max(y_rect)+1 min(z_rect)-1 max(z_rect)+1])
%axis([0 21 -10 10 -10 10])

roty = [0 0 1; 0 1 0;-1 0 0];
rotz = [0 1 0;-1 0 0;0 0 1];

x_rect_pixels = zeros(1,4);
y_rect_pixels = zeros(1,4);

for i = 1:4
   X = [x_rect(i); y_rect(i); z_rect(i)];
   X = rotz*X;
   Xh = 1/(X(3)+ xi*norm(X)) * X;
   Xh(3) = 1.0;
   
   xh_ic = K*Xh;
   
   x_rect_pixeles(i) = xh_ic(1);
   y_rect_pixeles(i) = xh_ic(2);
end

p1 = round([x_rect_pixeles(1); y_rect_pixeles(1)])
p2 = round([x_rect_pixeles(2); y_rect_pixeles(2)])
p3 = round([x_rect_pixeles(3); y_rect_pixeles(3)])
p4 = round([x_rect_pixeles(4); y_rect_pixeles(4)])

figure()
plot(x_rect_pixeles, y_rect_pixeles, '*')
text(x_rect_pixeles, y_rect_pixeles, labels)
axis([0 x_pixels, 0 y_pixels])

%% A partir del punto p1 se recupera su ubicacion sobre la esfera unitaria
x_ic = [p1;1];
xh = K\x_ic;
x = xh(1);
y = xh(2);

gamma = sqrt(1+ (1-xi^2)*(x^2 + y^2));
eta = (-gamma - xi*(x^2+y^2))/(xi^2*(x^2 + y^2) - 1);

xbar = [x; y; 1/(1+xi*eta)];

Xc = (eta\1 + xi)*xbar;
Xc = rotz'*Xc

figure()
plot3(x_rect, y_rect, z_rect, '*')
text(x_rect, y_rect, z_rect,labels)
grid on
xlabel('x')
ylabel('y')
zlabel('z')
axis([0 max(x_rect)+1 min(y_rect)-1 max(y_rect)+1 min(z_rect)-1 max(z_rect)+1])


%figure()
hold on
plot3([0, Xc(1)], [0, Xc(2)], [0, Xc(3)])

p1_3D = [x_rect(1); y_rect(1); z_rect(1)]
Xc_real = p1_3D./norm(p1_3D)