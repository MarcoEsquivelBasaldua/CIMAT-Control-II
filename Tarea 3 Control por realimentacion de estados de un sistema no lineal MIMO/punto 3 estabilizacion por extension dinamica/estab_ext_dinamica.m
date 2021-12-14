function [X , Y, U, T] = estab_ext_dinamica(xi, vi, t_end, dt)

X = []; Y = []; U = []; T = [];

x = xi(1);
y = xi(2);
theta = xi(3);

k = 5*[1 1 1 1]';
for i = 0:dt:t_end
    T = [T i];
    
    if i == 0
        z11 = x;
        z12 = vi*cos(theta);
        z21 = y;
        z22 = vi*sin(theta);
    else
        z11 = z(1);
        z12 = z(2);
        z21 = z(3);
        z22 = z(4);
    end
    
    z = [z11 z12 z21 z22]';
    
    f = [z12 0 z22 0 ]';
    g = [0 0;cos(theta) -z22; 0 0; sin(theta) z12];
    Gr = [g(2,:); g(4,:)];
    
    u = Gr\[-k(1)*z11-k(2)*z12; -k(3)*z21-k(4)*z22];
    U = [U u];
    
    z = z + dt*(f + g*u);
    
    x = z(1);
    y = z(3);
    theta = atan2(z12,z22);
    
    xi = [x y theta]';
    X = [X xi];
    
    yi = [x y]';
    Y = [Y yi];
end

end