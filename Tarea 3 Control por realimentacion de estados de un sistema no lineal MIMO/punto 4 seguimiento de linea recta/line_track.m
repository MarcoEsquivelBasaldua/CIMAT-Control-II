function [X, Y, U, T] = line_track(xi, Yf, l, tau, dt)

X = []; Y = []; U = []; T = [];

x0 = xi(1) + l*cos(xi(3));
y0 = xi(2) + l*sin(xi(3));

xf = Yf(1);
yf = Yf(2);

a = xf - x0;
b = yf - y0;

k = 0.8;
for t = 0:dt:tau
    T = [T t];
    
    x = xi(1);
    y = xi(2);
    theta = xi(3);
    X = [X xi];
    
    p = t/tau;
    Y_des = [x0+p*a; y0+p*b];
    Ydot_des = [a/tau; b/tau];
    
    g = [cos(theta) 0; sin(theta) 0; 0 1];
    h = [x + l*cos(theta); y + l*sin(theta)];
    Y = [Y h];
    
    Lgh = [cos(theta) sin(theta);-l*sin(theta) l*cos(theta)]';
    
    u = Lgh\(Ydot_des + k*(Y_des-h));
    U = [U u];
    
    xi = xi + dt*(g*u);
    
end

end