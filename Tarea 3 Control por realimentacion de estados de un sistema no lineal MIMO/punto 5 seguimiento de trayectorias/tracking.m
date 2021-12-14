function [X, Y, U, E, T] = tracking(xi, l, tau, dt)

X = []; Y = []; U = []; E = []; T = [];

k = 2.3;
for t = 0:dt:tau
    T = [T t];
    
    x = xi(1);
    y = xi(2);
    theta = xi(3);
    X = [X xi];
    
    Y_des = [10*cos(pi*t/5)+5*sin(pi*t/10); 10*sin(pi*t/10)-5*cos(pi*t/10)+5];
    Ydot_des = [-2*pi*sin(pi*t/5)+pi/2*cos(pi*t/10); pi*cos(pi*t/10)+pi/2*sin(pi*t/10)];
    
    g = [cos(theta) 0; sin(theta) 0; 0 1];
    h = [x + l*cos(theta); y + l*sin(theta)];
    Y = [Y h];
    
    Lgh = [cos(theta) sin(theta);-l*sin(theta) l*cos(theta)]';
    e = h - Y_des;
    E = [E e];
    
    u = Lgh\(Ydot_des + k*(Y_des-h));
    U = [U u];
    
    xi = xi + dt*(g*u);
    
end

end