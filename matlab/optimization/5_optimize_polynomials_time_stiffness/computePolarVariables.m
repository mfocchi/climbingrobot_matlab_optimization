function [theta, phi, l] = computePolarVariables(p)

 l = sqrt(p(1)^2+ p(2)^2+p(3)^2);
 phi = atan2(p(2), p(1));
 theta = acos(-p(3) /l);
 %theta = atan2(sqrt(p(1)^2+ p(2)^2), -p(3));

end