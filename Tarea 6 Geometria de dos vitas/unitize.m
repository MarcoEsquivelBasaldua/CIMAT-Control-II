function [x, y] = unitize(x1, y1)

n = sqrt(x1^2 + y1^2);
x = x1/n;
y = y1/n;

end