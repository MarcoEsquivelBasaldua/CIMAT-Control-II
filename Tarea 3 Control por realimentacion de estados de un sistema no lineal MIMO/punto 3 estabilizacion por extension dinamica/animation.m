function animation(X, T, dt)
close all
l = 0.5;

x = X(1,:);
y = X(2,:);
theta = X(3,:);


min_x = min(x) - l;
max_x = max(x) + l;
min_y = min(y) - l;
max_y = max(y) + l;

figure
for i = 1:length(T)
    ls_theta = l/2*sin(theta(i));
    lc_theta = l/2*cos(theta(i));
    
    p1 = [x(i)+ls_theta; y(i)-lc_theta];
    p2 = [x(i)-ls_theta; y(i)+lc_theta];
    
    plot([p1(1),p2(1)],[p1(2), p2(2)],'b-o', x(i),y(i),'r*')
    title ('Trayectoria del robot al origen')
    xlabel('X')
    ylabel('Y')
    axis equal
    axis([min_x max_x min_y max_y])
    grid on
    hold on
    
    pause(dt)
    exportgraphics(gcf,'RobotTracking.gif','Append',true);
end
hold off


figure
for i = 1:length(T)
    ls_theta = l/2*sin(theta(i));
    lc_theta = l/2*cos(theta(i));
    
    p1 = [x(i)+ls_theta; y(i)-lc_theta];
    p2 = [x(i)-ls_theta; y(i)+lc_theta];
    
    plot([p1(1),p2(1)],[p1(2), p2(2)],'b-o', x(i),y(i),'r*')
    title (sprintf('Ubicacion del robot en el instante %6.2f sec',i*dt))
    xlabel('X')
    ylabel('Y')
    axis equal
    axis([min_x max_x min_y max_y])
    grid on
    
    pause(dt)
    exportgraphics(gcf,'RobotPosition.gif','Append',true);
end

end