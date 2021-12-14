function Lo = ptFeaturesInteractionMatrix(vecDesired, vecDepthDesired, foc)

n = length(vecDesired)/2;
Lo = [];

z = vecDepthDesired(1);

for i= 1:1:n    
    u = vecDesired(2*i-1);
    v = vecDesired(2*i);
    
    Lo_i = [-foc/z, 0, u/z, u*v/z, -(foc^2+u^2)/foc, v;
            0, -foc/z, v/z, (foc^2+v^2)/foc -u*v/foc -u];
    
    Lo = [Lo; Lo_i];
end

end