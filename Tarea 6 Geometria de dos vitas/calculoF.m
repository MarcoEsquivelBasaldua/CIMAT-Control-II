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

% Bucle de entrada de al menos ocho pares de puntos
for i=1:12
   % capturamos el punto en la imagen uno
   figure(h1); [x1i,y1i] = ginput(1);
   
   % pintamos un circulo
   figure(h1); plot(x1i,y1i,'go','LineWidth',2);
   text(x1i,y1i,[' ' int2str(i)],'Erase','back','Color',[0 1 0]);
   
   % Normalizo
   p1=N1*[x1i;y1i;1];              
     
   %capturamos el punto en la imagen dos
   figure(h2); [x2i,y2i] = ginput(1);
   
   % pintamos un circulo
   figure(h2); plot(x2i,y2i,'ro','LineWidth',2);
   text(x2i,y2i,[' ' int2str(i)],'Erase','back','Color',[1 0 0]);

   % Normalizo
   p2 = N2*[x2i;y2i;1];   %

   
   % AÑADE A LA MATRIZ DEL SISTEMA LAS ECUACIONES QUE PROPORCIONA
   %  LA CORRESPONDENCIA DETECTADA EN ESTA ITERACION DEL BUCLE
   x1 = p1(1);
   y1 = p1(2);
   
   x2 = p2(1);
   y2 = p2(2);
   M=[M;
      x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1];
end

disp('Fin de entrada de datos')

% RESOLVER EL SISTEMA DE ECUACIONES DEFINIDO POR LA
%   MATRIZ M Y CALCULAR F
[u, s, v] = svd(M);
f = v(:,9);
F = reshape(f,3,3)';


% FORZAR A QUE EL RANGO DE F SEA 2
[u, s, v] = svd(F);
FF= u * diag([s(1,1) s(2,2) 0]) * v';
%FF= u*s*v';

% Desnormalizar, para tener la F en pixeles
F=N2'*FF*N1;

disp('Ubicacion del epipolo por espacio nulo de F')
e2 = null(F','r')


%Preparando rectas limite de la imagen (en proyectivo)para dibujar rectas epipolares
ejex=[0,1,0];
ejey=[1,0,0];
ejexlimite=cross([nx2,ny2,1],[0,ny2,1]);
ejeylimite=cross([nx2,ny2,1],[nx2,0,1]);

% Calculamos unas cuantas rectas epipolares
for i=1:5
   % capturamos el punto en la imagen uno
   figure(h1); [x1i,y1i] = ginput(1);
   
   % pintamos un circulo
   figure(h1); plot(x1i,y1i,'ro','LineWidth',2);
   
   %%%%%%%% CALCULA LA RECTA EPIPOLAR CORRESPONDIENTE EN LA IMAGEN 2
   R = F * [x1i,y1i,1]';
   
   %Interseccion de la recta con limites imagen
   pi1=cross(R,ejex);
   pi2=cross(R,ejey);
   
   % dibujamos la linea en la imagen 2
   if (pi1(1)/pi1(3))>=0
       pd1=pi1;
   else
       pd1=cross(R,ejexlimite);
   end
   
   if (pi2(2)/pi2(3))>=0
       pd2=pi2;
   else
       pd2=cross(R,ejeylimite);
   end
   
   figure(h2); plot([pd1(1)/pd1(3), pd2(1)/pd2(3)],[pd1(2)/pd1(3), pd2(2)/pd2(3)], 'g');
   
   % POSICION DEL EPIPOLO
   % Por la interseccion de rectas
   if i == 1
       R1 = R;
   elseif i == 2
       A = [R1(1), R1(2); R(1),R(2)];
       b = -[R1(3);R(3)];
       
       disp('Ubicacion del epipolo por interseccion de rectas')
       e2 = A\b
       
       figure(h2); plot(e2(1),e2(2), 'mo')
   end
       
   
end
