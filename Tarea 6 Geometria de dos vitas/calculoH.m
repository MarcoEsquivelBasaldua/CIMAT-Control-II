clf, close all;

% Leo las imagenes y las coloco en la matriz
I1=imread('EsquinaVistaDesdeCamaraTripie_640x480.jpg');
I2=imread('EsquinaYCamaraSobreTripie_640x480.jpg');

nx=size(I1,2);
ny=size(I1,1);

nx2=size(I2,2);
ny2=size(I2,1);

% Matriz normalizacion (al centro y anchura y altura uno)
N1=[1/nx 0 -1/2; 0 1/ny -1/2; 0 0 1];
N2=[1/nx2 0 -1/2; 0 1/ny2 -1/2; 0 0 1];


% Visualizo dichas imagenes

h1=figure;
h2=figure;
figure(h1);
imshow(I1);
hold on
figure(h2);
imshow(I2);
hold on;

M=[];

disp('Comienzo de entrada de datos')

% Bucle de entrada de los pares de puntos para calcular una Homografia
for i=1:4
   % capturamos el punto en la imagen uno
   figure(h1); [x1i,y1i] = ginput(1);
   
   % pintamos un circulo
   figure(h1);plot(x1i,y1i,'go','LineWidth',2);
   text(x1i,y1i,[' ' int2str(i)],'Erase','back','Color',[0 1 0]);
   
   % Normalizo
   p1=N1*[x1i;y1i;1];              
     
   %capturamos el punto en la imagen dos
   figure(h2);[x2i,y2i] = ginput(1);
   
   % pintamos un circulo
   figure(h2);plot(x2i,y2i,'ro','LineWidth',2);
   text(x2i,y2i,[' ' int2str(i)],'Erase','back','Color',[1 0 0]);

   % Normalizado del punto 2
   p2 = N2*[x2i;y2i;1];   %
   
   % AÑADE A LA MATRIZ DEL SISTEMA LAS ECUACIONES QUE PROPORCIONA
   %  LA CORRESPONDENCIA DETECTADA EN ESTA ITERACION DEL BUCLE
   x = p1(1);
   y = p1(2);
   x_prime = p2(1);
   y_prime = p2(2);
   
   M=[M;
      x, y, 1, 0, 0, 0, -x*x_prime, -y*x_prime, -x_prime;
      0, 0, 0, x, y, 1, -x*y_prime, -y*y_prime, -y_prime];
end

disp('Fin de entrada de datos')


% RESOLVER EL SISTEMA DE ECUACIONES DEFINIDO POR LA
%   MATRIZ M Y CALCULAR H
[u, s, v] = svd(M);
h = v(:,9);
Hn = reshape(h,3,3)';

% Deshacer la normalizacion
H = N2\Hn*N1;


% Verificacion de la homografia mediante transferencia de puntos
%   desde la imagen 1 hacia la imagen 2
for i=1:5
   % capturamos el punto en la imagen uno
   figure(h1); [x1i,y1i] = ginput(1);
   
   % pintamos un circulo en imagen 1
   figure(h1); plot(x1i,y1i,'ro','LineWidth',2);
   
   % CALCULA AQUI EL PUNTO TRANSFERIDO
   p = H*[x1i,y1i,1]';
   x2 = p(1)/p(3);  
   y2 = p(2)/p(3);
   
   % Pintamos el punto transferido
   figure(h2); plot(x2,y2,'bx','MarkerSize',12,'LineWidth',2);
   
   
end
