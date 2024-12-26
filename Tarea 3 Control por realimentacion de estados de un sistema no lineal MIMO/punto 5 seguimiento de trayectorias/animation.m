function animation(X, Y, T, dt)
close all
l = 0.5;

x = X(1,:);
y = X(2,:);
theta = X(3,:);

xp = Y(1,:);
yp = Y(2,:);

X0 = Y(:,1);
Xf = Y(:,end);

min_x = min(min(x), min(xp)) - l;
max_x = max(max(x), max(xp)) + l;
min_y = min(min(y), min(yp)) - l;
max_y = max(max(y), max(yp)) + l;

traj = [];
for t = 0:dt:T(end)
    Y_des = [10*cos(pi*t/5)+5*sin(pi*t/10); 10*sin(pi*t/10)-5*cos(pi*t/10)+5];
    traj = [traj Y_des];
end


figure
for t = 1:length(T)    
    ls_theta = l/2*sin(theta(t));
    lc_theta = l/2*cos(theta(t));
    
    p1 = [x(t)+ls_theta; y(t)-lc_theta];
    p2 = [x(t)-ls_theta; y(t)+lc_theta];
    
    if mod(t,5) == 0
        plot(traj(1,:), traj(2,:), 'g-', [p1(1),p2(1)],[p1(2), p2(2)],'b-o', [x(t),xp(t)], [y(t),yp(t)],'r-')
        title ('Trayectoria del robot al origen')
        xlabel('X')
        ylabel('Y')
        axis equal
        axis([min_x max_x min_y max_y])
        grid on
        hold on
    end
         
    pause(dt)
    exportgraphics(gcf,'RobotTracking.gif','Append',true);
end
hold off


figure
for t = 1:length(T)
    ls_theta = l/2*sin(theta(t));
    lc_theta = l/2*cos(theta(t));
    
    p1 = [x(t)+ls_theta; y(t)-lc_theta];
    p2 = [x(t)-ls_theta; y(t)+lc_theta];
    
    plot(traj(1,:), traj(2,:), 'g-', [x(t),xp(t)], [y(t),yp(t)],'r-', [p1(1),p2(1)],[p1(2), p2(2)],'b-o')
    title (sprintf('Ubicacion del robot en el instante %6.2f',t*dt))
    xlabel('X')
    ylabel('Y')
    axis equal
    axis([min_x max_x min_y max_y])
    grid on
    
    pause(dt)
    exportgraphics(gcf,'RobotPosition.gif','Append',true);
end

end