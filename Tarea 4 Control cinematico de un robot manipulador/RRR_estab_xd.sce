clc, xdel(winsid()),

//Robot links' lenghts
d1=1; d2=0.8; d3=0.4;
//Initial angles
q0=[0.2;0.2;-0.2]; q=q0;
//Sampling time
dt=0.05;
da=[0;0;0];

//Robot definition
L1 = Link('d', 0, 'a', d1, 'alpha', 0)
L2 = Link('d', 0, 'a', d2, 'alpha', 0)
L3 = Link('d', 0, 'a', d3, 'alpha', 0)
L = list(L1,L2,L3)
bot = SerialLink(L, 'name', 'my robot') //[L1,L2,L3]

//Estimation of Cartesian pose from initial configuration
T0=fkine(bot,q0)
x0=[T0(1,4); T0(2,4); atan(T0(2,1),T0(1,1))]
x=x0

//Controller parameters
xd=[0.8; 1.5; 0.3];
k=eye(3,3); k(1,1)=0.5; k(2,2)=0.5; k(3,3)=0.5;

xAr=[]; xdAr=[]; eAr=[]; qAr=[]; daAr=[]; tAr=[];

for t=0:dt:10

plot_robot(bot,q');
a=gca(); a.data_bounds=[-3,-3,-1;3,3,3];
a.view="2d";
sleep(100)

//Jacobian computation
s1=sin(q(1)); s12=sin(q(1)+q(2)); s123=sin(q(1)+q(2)+q(3));
c1=cos(q(1)); c12=cos(q(1)+q(2)); c123=cos(q(1)+q(2)+q(3));
J=[-d1*s1-d2*s12-d3*s123 -d2*s12-d3*s123 -d3*s123;
    d1*c1+d2*c12+d3*c123  d2*c12+d3*c123  d3*c123
    1                     1               1];

//Integration of dynamic model to obtain new pose
x=x+dt*J*da

e=x-xd; //Cartesian error
da=inv(J)*(-k*e); //Controller (joint's velocities)

q=q+dt*(da); //Rough estimation of new q

//Correct estimation of new q
//T=[cos(x(3)) -sin(x(3)) 0  x(1);
//   sin(x(3))  cos(x(3)) 0  x(2);
//   0          0         1  0;
//   0          0         0  1]
//q = ikine(bot,T,q0,[1 1 0 0 0 1])

xAr=[xAr x]; xdAr=[xdAr xd]; eAr=[eAr e]; 
qAr=[qAr q]; daAr=[daAr da]; tAr=[tAr t];

end
figure(); plot(tAr,xdAr,'--',tAr,xAr), title('End-effector position and orientation')
figure(); plot(tAr,eAr), title('Carterian error')
figure(); plot(tAr,daAr), title('Joints velocities')
figure(); plot(tAr,qAr), title('Configuration angles')
