function animation(X, Y, T, dt)
close all
l = 0.5;

x = X(1,:);
y = X(2,:);
theta = X(3,:);

xp = Y(1,:);
yp = Y(2,:);

min_x = min(min(x), min(xp)) - l;
max_x = max(max(x), max(xp)) + l;
min_y = min(min(y), min(yp)) - l;
max_y = max(max(y), max(yp)) + l;

figure
for i = 1:length(T)
    ls_theta = l/2*sin(theta(i));
    lc_theta = l/2*cos(theta(i));
    
    p1 = [x(i)+ls_theta; y(i)-lc_theta];
    p2 = [x(i)-ls_theta; y(i)+lc_theta];
    
    plot([x(i),xp(i)], [y(i),yp(i)],'r-', [p1(1),p2(1)],[p1(2), p2(2)],'b-o')
    title ('Trayectoria del robot al origen')
    xlabel('X')
    ylabel('Y')
    axis equal
    axis([min_x max_x min_y max_y])
    grid on
    hold on
    
    pause(dt)
end
hold off


figure
for i = 1:length(T)
    ls_theta = l/2*sin(theta(i));
    lc_theta = l/2*cos(theta(i));
    
    p1 = [x(i)+ls_theta; y(i)-lc_theta];
    p2 = [x(i)-ls_theta; y(i)+lc_theta];
    
    plot([x(i),xp(i)], [y(i),yp(i)],'r-', [p1(1),p2(1)],[p1(2), p2(2)],'b-o')
    title (sprintf('Ubicacion del robot en el instante %6.2f',i*dt))
    xlabel('X')
    ylabel('Y')
    axis equal
    axis([min_x max_x min_y max_y])
    grid on
    
    pause(dt)
end

end