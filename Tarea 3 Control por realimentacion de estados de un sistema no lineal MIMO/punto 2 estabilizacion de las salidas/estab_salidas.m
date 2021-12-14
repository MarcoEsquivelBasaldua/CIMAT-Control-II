function [X, Y, U, T] = estab_salidas(xi, l, t_end, dt)

X = []; Y = []; U = []; T = [];

k = 2;
for i = 0:dt:t_end
    T = [T i];
    
    x = xi(1);
    y = xi(2);
    theta = xi(3);
    X = [X xi];
    
    g = [cos(theta) 0; sin(theta) 0; 0 1];
    h = [x + l*cos(theta); y + l*sin(theta)];
    Y = [Y h];
    
    Lgh = [cos(theta) sin(theta);-l*sin(theta) l*cos(theta)]';
    
    u = Lgh\(-k*h);
    U = [U u];
    
    xi = xi + dt*(g*u);
    
end

end