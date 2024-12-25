clc, clear, close all

% Robot DH parameters
a = [0, 0.4318, 0, 0, 0, 0];
alpha = [pi/2, 0, -pi/2, pi/2, -pi/2, 0];
d = [0.6604, -0.2286, 0.072, 0.4318, 0, 0];

% Initial angles
q0 = zeros(6,1)-0.5;
q = q0;

l(1) = Link([q(1), d(1), a(1), alpha(1), 0]);

for i = 1:6
    l(i) = Link([q(i), d(i), a(i), alpha(i), 0]);
end

puma = SerialLink(l, 'name', 'puma560');

%Sampling time
dt = 0.05;
da = zeros(6,1);

%Estimation of Cartesian pose from initial configuration
T0 = puma.fkine(q0);
p0 = [T0.t(1); T0.t(2); T0.t(3)]
p = p0;

%Controller parameters
pd = [0.3; -0.2; 1.0];
k=eye(3,3); k(1,1)=0.5; k(2,2)=0.5; k(3,3)=0.5;

pAr=[]; pdAr=[]; eAr=[]; qAr=[]; daAr=[]; tAr=[];

lim_axis = 1.8;
for t = 0:dt:10.
   plot_sphere(pd, 0.05, 'y');
   puma.plot(q')
   axis([-lim_axis lim_axis -lim_axis lim_axis 0 lim_axis])
    
    % Numeric Jacobian computation
    J = puma.jacob0(q);
    J = J(1:3,:);
    
    %Integration of dynamic model to obtain new pose
    %x = x + dt*J*da
    T = puma.fkine(q);
    p = [T.t(1); T.t(2); T.t(3)]

    e = p - pd; %Cartesian error
    da = pinv(J)*(-k*e); %Controller (joint's velocities)
    
    q = q + dt*da; %Rough estimation of new q    
    T = puma.fkine(q);
    q = puma.ikine(T)';
    

    pAr=[pAr p]; pdAr=[pdAr pd]; eAr=[eAr e]; 
    qAr=[qAr q]; daAr=[daAr da]; tAr=[tAr t];

    exportgraphics(gcf,'PumaSinglePoseUSingJacobFunction.gif','Append',true);
end

figure(); 
subplot(2,2,1)
plot(tAr,pAr,tAr,pdAr,'--'), title('End-effector position and orientation'), legend('x','y','z'), grid on
subplot(2,2,2)
plot(tAr,eAr, tAr,zeros(size(tAr)),'k'), title('Cartesian error'), legend('e_x','e_y','e_z'), grid on
subplot(2,2,3)
plot(tAr,qAr), title('Configuration angles'), legend('\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6'), grid on
subplot(2,2,4)
plot(tAr,daAr), title('Joints velocities'), legend('\theta_1 dot','\theta_2 dot','\theta_3 dot','\theta_4 dot','\theta_5 dot','\theta_6 dot'), grid on