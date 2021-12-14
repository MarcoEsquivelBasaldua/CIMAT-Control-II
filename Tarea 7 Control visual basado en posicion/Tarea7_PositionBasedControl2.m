clear all; close all; clc;

nPoints = 8;
foc=0.002; %Focal de la camara
%Defining camera poses.
%Target image
lon_g=-0.6; lat_g=0.0; ver_g=0.; pitch=-0*pi/180; yaw=0*pi/180; roll=0*pi/180;
TT = transl(lat_g,-ver_g,lon_g)*trotx(pitch)*troty(yaw)*trotz(roll);
camT = CentralCamera('name', 'T1', 'default', 'focal', foc, 'pose', TT);
%Current image
%lon=-0.8; lat=-0.5; ver=0.0; pitch=-0*pi/180; yaw=0*pi/180; roll=10*pi/180;
lon=-0.8; lat=-0.5; ver=0.6; pitch=-10*pi/180; yaw=30*pi/180; roll=10*pi/180;
%lon=-0.9; lat=0.3; ver=-0.8; pitch=30*pi/180; yaw=10*pi/180; roll=40*pi/180;
%lon=-0.6; lat=0.01; ver=0.0; pitch=0*pi/180; yaw=0*pi/180; roll=90*pi/180;

TC = transl(lat,-ver,lon)*trotx(pitch)*troty(yaw)*trotz(roll);
camC = CentralCamera('name', 'TC', 'default', 'focal', foc, 'pose', TC);
camCs=camC;

%Definition of 3D points
Tgrid = transl(0,0.0,1)*trotx(0)*troty(0);
P1 = mkgrid(2, 1.0, 'T', Tgrid);
P2 = [P1(1:2,:); ones(1,nPoints/2)];
P = [P1 P2];
%Projection of points on each camera
pT = camT.project(P);
pC = camC.project(P);
pS = pC;

%============== Simulation Initialization ==============%

%%%Timing parameters%%%
dt = 0.01;     % Time Delta, seconds.
t0 = 0; %Start time of the simulation
t1 = 20.0;%End time of the simulation

%%%Initial controls%%%
v       = [0;0;0];       %speed in m/s
omega   = [0;0;0];   %angular velocity in rad/s
U = [v;omega];

%%%Variables initialization%%%
steps = (t1-t0)/dt + 1; %Quantity of simulation steps

UArray = [];           %Matrix to save controls history
tArray = [];           %Matrix to save the time steps
pixelCoordsArray = []; %Matrix to save points positions
averageErrorArray = []; %Matrix to save points positions
positionArray = [];

I = eye(3,3);
%Definition of control gains
Gain = 1.2*ones(6,1);

%=============Simulation Loop==============%
h = 20;
epsilon = 10;
hist_error = zeros(h,1);
t = t0;
for j = 1: 1 : steps
    

       %---Calculate new translation and rotation values using Euler's method   
       lat=lat+dt*U(1,1); ver=ver+dt*U(2,1); lon=lon+dt*U(3,1);
       pitch=pitch+dt*U(4,1); yaw=yaw+dt*U(5,1); roll=roll+dt*U(6,1);
       TC = transl(lat,-ver,lon)*trotx(pitch)*troty(yaw)*trotz(roll);
       camC = CentralCamera('name', 'TC', 'default', 'focal', foc, 'pose', TC, 'centre', [512;512]);
       
       
       %
       if j == 1
           U_last = U;
       else
           for i =1:1:6
               if abs(U_last(i) - U(i)) > abs(U_last(i) + U(i))
                   U(i) = -U(i);
               end
           end
           
           U_last = U;
       end
       %
       %{
       if j == 1
           U_last = U;
       else
           if norm(U_last(1:3) - U(1:3)) > norm(U_last(1:3) + U(1:3))
               U(1:3) = -U(1:3);
           end
           if norm(U_last(4:6) - U(4:6)) > norm(U_last(4:6) + U(4:6))
               U(4:6) = -U(4:6);
           end
           
           U_last = U;
       end
       %}
       
       
       %%%Save values in the arrays%%%
       tArray = [tArray t];   UArray = [UArray U];

       pC = camC.project(P);
       pixelCoordsArray = [pixelCoordsArray pC(:)];

    %  ---POSITION BASED CONTROL COMPUTATION---------------------------------------      
       %1)Calcular matriz fundamental a partir de pT y pC
       F = fmatrix(pT, pC);
       %2)Calcular matriz escencial
       E = camC.E(F);
       %3)Descomponer la matriz escencial
       s = camC.invE(E, P);
       %4)Formular los errores de retroalimentación ev y ew
       %ev=[lat-lat_g -ver+ver_g lon-lon_g]';  ew=[pitch yaw roll]';
       ev=[lat -ver lon]';  ew=[pitch yaw roll]';
       e = [ev; ew];

       position=[lat -ver lon]';
      
       
       %{
       rot = [pitch yaw roll]';
       if j == 1
           position_last = position;
           rotation_last = rot;
       else
           if norm(position_last - position) > norm(position_last + position)
               position = -position;
           end
           if norm(rotation_last - rot) > norm(rotation_last + rot)
               rot = -rot;
           end
           
           position_last = position;
           rotation_last = rot;
       end
       
       ev = position;  ew=rot;
       e = [ev; ew];
       %}
       
       
       positionArray=[positionArray position];  

    %  ---Average feature error
       for k =1:nPoints
          errorPixels(k,1) =  norm(pC(:,k)-pT(:,k));
       end
       averageError = mean(errorPixels);
       averageErrorArray = [averageErrorArray averageError];
       if averageError<5, break; end
       
       if j <= h 
           hist_error(j) = averageError;
       else
           hist_error(1:h-1) = hist_error(2:h);
           hist_error(h) = averageError;
           
           if averageError - mean(hist_error) > epsilon
               break;
           end
       end
       
    %  ------------------------------------------------------------------------   

    %---Constant control U
       %U = zeros(6,1);
       %U(6) = -0.1;
       
    %---Defining control law
       R = s(1).R;
       tr = averageError*s(1).t;
       %
       r11 = R(1,1);
       r12 = R(1,2);
       r13 = R(1,3);
       r21 = R(2,1);
       r22 = R(2,2);
       r23 = R(2,3);
       r31 = R(3,1);
       r32 = R(3,2);
       r33 = R(3,3);
       theta = real(acos(0.5*(r11+r22+r33 - 1)));
       utheta = theta/(2*sin(theta)).*[r32-r23; r13-r31; r21-r12];
       %
       
       %[utheta, theta] = rotm2axang(R);
       
       I3 = eye(3);
       t_skew = skew(tr);
       L_thetau = I3 - theta/2*skew(utheta') + (1-(theta*sin(theta))/(4*sin(theta/2)^2))*skew(utheta')^2;
       %L_thetau = I3 - theta/2*skew(utheta') + (1-(sinc(theta)/sinc(theta/2)^2))*skew(utheta')^2;
       
       %Le_1_hat = [-I3 t_skew\L_thetau;zeros(3) inv(L_thetau)];
       Le = [R zeros(3); zeros(3) L_thetau];
       U = -Gain.* (Le\e);
       U(2,1)= -U(2,1);  %Hay necesidad de negar la velocidad en vertical

       %%%Update time%%%
       t = t + dt;
       
       % Check the right control
       %{
       if j == 1
           U_last = U;
       else
           if norm(U_last - U) > norm(U_last + U)
               U = -U;
           end
           
           U_last = U;
       end
       %}
       %{
       if j == 1
           U_last = U;
       else
           if norm(U_last(1:3) - U(1:3)) > norm(U_last(1:3) + U(1:3))
               U(1:3) = -U(1:3);
           end
           if norm(U_last(4:6) - U(4:6)) > norm(U_last(4:6) + U(4:6))
               U(4:6) = -U(4:6);
           end
           
           U_last = U;
       end
       %}
       
       % Check the right control
       %{
       if j == 1
           U_last = U;
       else
           for i =1:1:6
               if norm(U_last(i) - U(i)) > norm(U_last(i) + U(i))
                   U(i) = -U(i);
               end
           end
           
           U_last = U;
       end
       %}
       
end
%==========================================%

%=============Display Results==============%
figure(1);
subplot(2,2,1)
axis([-1 1 -1 1 -1.8 1.2]); grid on;
camCs.plot_camera('color', 'r', 'label');
camT.plot_camera('color', 'g', 'label');
camC.plot_camera('color', 'r', 'label');
plot_sphere(P, 0.02, 'b'); hold on;
plot3(positionArray(1,:),positionArray(2,:),positionArray(3,:),'m-','LineWidth',2);
text(P(1,1),P(2,1),P(3,1),'p1'); text(P(1,3),P(2,3),P(3,3),'p3');

%===Plot image points and their corresponding trayectories===%
subplot(2,2,2)
hold on;
%---Starting point positions in the image
for i = 1 : nPoints
    plot(pS(1,i), pS(2,i), 'Marker','o', 'MarkerEdgeColor','k','MarkerFaceColor','g');
end
%---Desired point positions in the image
for i = 1 : nPoints
    plot(pT(1,i), pT(2,i), 'Marker','o', 'MarkerEdgeColor','k','MarkerFaceColor','b'); 
end
%---Final position of the points in the image after simulation
for i = 1 : nPoints
    plot(pC(1,i), pC(2,i), 'Marker','o', 'MarkerEdgeColor','k','MarkerFaceColor','r');
end
plot(pixelCoordsArray(1,:), pixelCoordsArray(2,:),'LineStyle','--');
plot(pixelCoordsArray(3,:), pixelCoordsArray(4,:),'LineStyle','--');
plot(pixelCoordsArray(5,:), pixelCoordsArray(6,:),'LineStyle','--');
plot(pixelCoordsArray(7,:), pixelCoordsArray(8,:),'LineStyle','--');
plot(pixelCoordsArray(9,:), pixelCoordsArray(10,:),'LineStyle','--');
plot(pixelCoordsArray(11,:), pixelCoordsArray(12,:),'LineStyle','--');
plot(pixelCoordsArray(13,:), pixelCoordsArray(14,:),'LineStyle','--');
plot(pixelCoordsArray(15,:), pixelCoordsArray(16,:),'LineStyle','--');
set(gca,'Xlim',[0 1024],'fontw','b')
set(gca,'Ylim',[0 1024],'fontw','b')
grid on;

%===Draw the controls trend===%
subplot(2,2,3)
hold on;
plot(tArray, UArray','LineWidth',1.5);
grid on;
legend('vx','vy','vz','wx','wy','wz');
xlabel('time (s)')
ylabel('Controls')
hold off

%===Average Error===%
subplot(2,2,4)
plot(tArray, averageErrorArray','LineWidth',1.5);
grid on;
legend('averageError');
xlabel('time (s)')
ylabel('Average Error')
hold off
