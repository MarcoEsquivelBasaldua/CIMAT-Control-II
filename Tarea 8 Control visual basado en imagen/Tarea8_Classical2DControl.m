clear all; close all; clc;

nPoints = 4;
foc=0.002; %Camera's focal lenght
%Camera Poses:
%Target pose
T1 = transl(0, 0, -0.3);
cam1 = CentralCamera('name', 'camera 1', 'default', 'focal', foc, 'pose', T1);
%Current pose
lon=-0.8; lat=-0.5; ver=0.0; pitch=-0*pi/180; yaw=0*pi/180; roll=10*pi/180;
%lon=-0.8; lat=-0.5; ver=0.6; pitch=-10*pi/180; yaw=30*pi/180; roll=10*pi/180;
%lon=-0.9; lat=0.3; ver=-0.8; pitch=30*pi/180; yaw=10*pi/180; roll=40*pi/180;


T2 = transl(lat,-ver,lon)*trotx(pitch)*troty(yaw)*trotz(roll);
cam2 = CentralCamera('name', 'camera 2', 'default', 'focal', foc, 'pose', T2);
camS = cam2;

Tgrid = transl(0,0.0,1)*trotx(0)*troty(0);
P = mkgrid(2, 1.0, 'T', Tgrid);
%Projecting the 3D points to generate target and current images
p1 = cam1.project(P);
p2 = cam2.project(P);
ps = p2;

%============== Simulation Initialization ==============%

%%%Timing parameters%%%
dt=0.01; %Time Delta, seconds.
t0=0;    %Start time
t1=15.0;  %(1)Define an adequate end time

%%%Initial controls%%%
v       = [0;0;0];   %speed in m/s
omega   = [0;0;0];   %angular velocity in rad/s
U = [v;omega];

%%%Variables initialization%%%
steps = (t1-t0)/dt + 1; %Quantity of simulation steps

UArray = [];            %Matrix to save controls history
tArray = [];            %Matrix to save the time steps
pixelCoordsArray = [];  %Matrix to save points positions
averageErrorArray = []; %Matrix to save points positions
positionArray = [];

I = eye(3,3);
Gain = eye(6,6);
%(2)Define an adequate control gain
lambda = 0.8;
Gain(1,1) = lambda;
Gain(2,2) = lambda;
Gain(3,3) = lambda;
Gain(4,4) = lambda;
Gain(5,5) = lambda;
Gain(6,6) = lambda;

t = t0;
zFix= -T1(3,4); %(3)Define an adequate estimated fixed depth for the target features
K1=cam1.K; K2=cam2.K; 
p1n=K1\[p1; ones(1,4)];
%Defining the desired features vector (s*)
vecDesired=[p1n(1:2,1); p1n(1:2,2); p1n(1:2,3); p1n(1:2,4)];
vecDepthDesired=zFix*ones(size(vecDesired));
%Defining the interaction matrix using the target image
Lo = ptFeaturesInteractionMatrix(vecDesired, vecDepthDesired, 1);
%Lo = ptFeaturesInteractionMatrix(p1, vecDepthDesired, foc);

%=============Simulation Loop==============%
for j = 1: 1 : steps
     
   %%%Save values in the arrays%%%
   tArray = [tArray t];
   UArray = [UArray U];
   
   %---Calculate new translation and rotation values using Euler's method   
   lat=lat+dt*U(1,1); ver=ver+dt*U(2,1); lon=lon+dt*U(3,1);
   pitch=pitch+dt*U(4,1); yaw=yaw+dt*U(5,1); roll=roll+dt*U(6,1);
   T2 = transl(lat,-ver,lon)*trotx(pitch)*troty(yaw)*trotz(roll);
   cam2 = CentralCamera('name', 'camera 2', 'default', 'focal', foc, 'pose', T2, 'centre', [512;512]);

   %(4)Project the points from the new current pose
   p2 = cam2.project(P);
   pixelCoordsArray = [pixelCoordsArray p2(:)];
   
   p2n=K1\[p2; ones(1,4)];
   %Defining the current features vector (s)
   vecCurrent=[p2n(1:2,1); p2n(1:2,2); p2n(1:2,3); p2n(1:2,4)];   
   
%  -----------CONTROL COMPUTATION---------------------------------------
    %(5)Define the error vector (e=s-s*)
    e = vecCurrent - vecDesired;
    %(6)Define the control law (U=-gain*pseudoinv(Lo)*e)
    U = -lambda.*pinv(Lo)*e;
    U(2,1)=-U(2,1); %Hay necesidad de negar la velocidad en vertical
  
   position=[lat -ver lon]';
   positionArray=[positionArray position];
%  ---Average feature error
   for k =1:4;
      errorPixels(k,1) =  norm(p2(:,k)-p1(:,k));
   end
   averageError = mean(errorPixels);
   averageErrorArray = [averageErrorArray averageError];   
   
   t = t + dt;
end

%=============Display Results==============%
figure();
h1 = subplot(2,2,1);
axis([-1 1 -1 1 -1 1.2]); grid on;
cam1.plot_camera('color', 'b', 'label');
cam2.plot_camera('color', 'r', 'label');
camS.plot_camera('color', 'g', 'label');
plot_sphere(P, 0.02, 'b'); hold on;
plot3(positionArray(1,:),positionArray(2,:),positionArray(3,:),'m-','LineWidth',2);

%===Plot image points and their corresponding trayectories===%
h2 = subplot(2,2,2);
hold on;
%---Desired point positions in the image
for i = 1 : nPoints
    plot(p1(1,i), p1(2,i), 'Marker','o', 'MarkerEdgeColor','k','MarkerFaceColor','b'); 
end
%---Starting point positions in the image
for i = 1 : nPoints
    plot(ps(1,i), ps(2,i), 'Marker','o', 'MarkerEdgeColor','k','MarkerFaceColor','g');
end
%---Final position of the points in the image after simulation
for i = 1 : nPoints
    plot(p2(1,i), p2(2,i), 'Marker','o', 'MarkerEdgeColor','k','MarkerFaceColor','r');
end
plot(pixelCoordsArray(1,:), pixelCoordsArray(2,:),'LineStyle','--');
plot(pixelCoordsArray(3,:), pixelCoordsArray(4,:),'LineStyle','--');
plot(pixelCoordsArray(5,:), pixelCoordsArray(6,:),'LineStyle','--');
plot(pixelCoordsArray(7,:), pixelCoordsArray(8,:),'LineStyle','--');
set(gca,'Xlim',[0 1024],'fontw','b')
set(gca,'Ylim',[0 1024],'fontw','b')

grid on;
hold off

%===Draw the controls trend===%
h3 = subplot(2,2,3);
hold on;
plot(tArray, UArray','LineWidth',1.5);
grid on;
legend('vx','vy','vz','wx','wy','wz');
xlabel('time (s)')
ylabel('Controls')
hold off

%===Average Error===%
h4 = subplot(2,2,4);
hold on;
plot(tArray, averageErrorArray','LineWidth',1.5);
grid on;
legend('averageError');
xlabel('time (s)')
ylabel('Average Error')
hold off
