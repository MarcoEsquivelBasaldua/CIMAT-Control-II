close all

a = [0, 0.4318, 0, 0, 0, 0];
alpha = [pi/2, 0, -pi/2, pi/2, -pi/2, 0];
d = [0.6604, -0.2286, 0.072, 0.4318, 0, 0];
q = zeros(1,6);

l(1) = Link([q(1), d(1), a(1), alpha(1), 0]);

for i = 1:6
    l(i) = Link([q(i), d(i), a(i), alpha(i), 0]);
end

puma = SerialLink(l, 'name', 'puma560');


lim_axis = 1.8;
puma.plot(q)
axis([-lim_axis lim_axis -lim_axis lim_axis 0 lim_axis])

%puma.teach()
