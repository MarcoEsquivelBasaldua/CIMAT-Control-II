clc, clear all, close all

%Robot links' lenghts
d1=1; d2=0.8; d3=0.4;
%Initial angles
q0=[0.2;0.2;-0.2]; q=q0;
%Sampling time
dt=0.05;
da=[0;0;0];

%Robot definition
L1 = Link('d', 0, 'a', d1, 'alpha', 0)
L2 = Link('d', 0, 'a', d2, 'alpha', 0)
L3 = Link('d', 0, 'a', d3, 'alpha', 0)
%L = list(L1,L2,L3)
%L=[];
bot = SerialLink([L1 L2 L3], 'name', 'my robot') %[L1,L2,L3]

%Estimation of Cartesian pose from initial configuration
T0=bot.fkine(q0)
x0=[T0.t(1); T0.t(2); atan2(T0.n(2),T0.n(1))]
x=x0

%Controller parameters
xd=[0.8; 1.5; 0.3];
k=eye(3,3); k(1,1)=0.5; k(2,2)=0.5; k(3,3)=0.5;

xAr=[]; xdAr=[]; eAr=[]; qAr=[]; daAr=[]; tAr=[];

for t=0:dt:10

bot.plot(q')

% Numeric Jacobian computation
J = bot.jacob0(q);
J = [J(1:2,:) ; J(end,:)];

%Integration of dynamic model to obtain new pose
x=x+dt*J*da

e=x-xd; %Cartesian error
da=inv(J)*(-k*e); %Controller (joint's velocities)

q=q+dt*(da); %Rough estimation of new q

%Correct estimation of new q
% T=[cos(x(3)) -sin(x(3)) 0  x(1);
%    sin(x(3))  cos(x(3)) 0  x(2);
%      0          0         1  0;
%      0          0         0  1]
% q = ikine(bot,T,q0,[1 1 0 0 0 1])'
%q = bot.ikine3(T)

xAr=[xAr x]; xdAr=[xdAr xd]; eAr=[eAr e]; 
qAr=[qAr q]; daAr=[daAr da]; tAr=[tAr t];

end
figure(); plot(tAr,xdAr,'--',tAr,xAr), title('End-effector position and orientation')
figure(); plot(tAr,eAr), title('Carterian error')
figure(); plot(tAr,daAr), title('Joints velocities')
figure(); plot(tAr,qAr), title('Configuration angles')