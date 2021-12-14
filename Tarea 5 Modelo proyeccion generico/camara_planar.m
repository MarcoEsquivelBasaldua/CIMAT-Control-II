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

K = [a_x 0 p_x;0 a_y p_y;0 0 1];

% Ubicacion de las esquinas del rectangulo
L = 2;
W = 2;
l = L/2;
w = W/2;
rect_center = [5,0,0]';

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

p1 = [x_rect_pixeles(1); y_rect_pixeles(1)]
p2 = [x_rect_pixeles(2); y_rect_pixeles(2)]
p3 = [x_rect_pixeles(3); y_rect_pixeles(3)]
p4 = [x_rect_pixeles(4); y_rect_pixeles(4)]

for i = 1:4
   X = [x_rect(i); y_rect(i); z_rect(i)];
   X = rotz*roty*X;
   Xh = 1/X(3) * X;
   
   xh_ic = K*Xh;
   
   x_rect_pixeles(i) = xh_ic(1);
   y_rect_pixeles(i) = xh_ic(2);
end

round(x_rect_pixeles)
round(y_rect_pixeles)

figure()
plot(x_rect_pixeles, y_rect_pixeles, '*')
text(x_rect_pixeles, y_rect_pixeles, labels)
axis([0 x_pixels, 0 y_pixels])