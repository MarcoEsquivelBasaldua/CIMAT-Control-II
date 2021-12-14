function [vec, theta] = rotm2axang(R)
% This function converts the rotation matrix R into axis-angle form
    vec=[0 0 0];
    theta = acos((trace(R)-1)/2);
    
    if theta < 0.001
        vec=[0 0 0];
    elseif sin(theta) < 0.001
        if (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
            x=sqrt((R(1,1)+1)./2);
            y=R(2,1)./(2.*x);
            z=R(3,1)./(2.*x);
        elseif (R(2,2) > R(1,1)) && (R(2,2) > R(3,3))
            y=sqrt((R(2,2)+1)./2);
            x=R(1,2)./(2.*y);
            z=R(3,2)./(2.*y);
        elseif (R(3,3) > R(1,1)) && (R(3,3) > R(2,2))
            z=sqrt((R(3,3)+1)./2);
            x=R(1,3)./(2.*z);
            y=R(2,3)./(2.*z);
        else
            x=sqrt((R(1,1)+1)./2);
            y=R(1,2)./(2.*x);
            z=R(1,3)./(2.*x);            
        end            
            
        %vec=[x y z;-x -y -z];
        %theta=[theta;theta];
        vec = [x y z];
        
    else
        vec(1,1)=(1/(2*sin(theta))).*(R(3,2)-R(2,3));
        vec(1,2)=(1/(2*sin(theta))).*(R(1,3)-R(3,1));
        vec(1,3)=(1/(2*sin(theta))).*(R(2,1)-R(1,2));
    end
    
    %axang = [vec, theta];
end