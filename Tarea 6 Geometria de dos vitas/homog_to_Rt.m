function [R1,t1,n1, R2,t2,n2, zeta] = homog_to_Rt(H)
[U,S,V] = svd(H);
s1 = S(1,1)/S(2,2);
s3 = S(3,3)/S(2,2);

zeta = s1-s3
a1 = sqrt(1-s3^2);
b1 = sqrt(s1^2-1);

[a,b] = unitize(a1,b1);
[c,d] = unitize( 1+s1*s3, a1*b1 );
[e,f] = unitize( -b/s1, -a/s3 );

v1 = V(:,1); v3 = V(:,3);
n1 = b*v1-a*v3;

n2 = b*v1+a*v3;
R1 = U*[c,0,d; 0,1,0; -d,0,c]*V';
R2 = U*[c,0,-d; 0,1,0; d,0,c]*V';

t1 = -R1*(e*v1+f*v3);
t2 = -R2*(e*v1-f*v3);
if (n1(3)<0) 
    t1 = -R1*t1; 
    n1 = -n1; 
end

if (n2(3)<0) 
    t2 = -R2*t2; 
    n2 = -n2; 
end

t1 = zeta*t1;
t2 = zeta*t2;
end
