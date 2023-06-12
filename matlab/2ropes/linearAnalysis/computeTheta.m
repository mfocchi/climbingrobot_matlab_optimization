function [psi] = computeRPY(p)


 psi = atan2(p(1), -p(3));

end