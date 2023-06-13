function [p,pd] = computePositionVelocity(psi, l1, l2, psid,l1d, l2d)

    if nargin >3
        for i=1:length(psi)    
            [px(i), py(i), pz(i), pdx(i), pdy(i), pdz(i)] = forwardKin(psi(i), l1(i),l2(i), psid(i), l1d(i),l2d(i));

        end 
        pd = [pdx;pdy;pdz];
    else
        for i=1:length(psi)    
            [px(i), py(i), pz(i)] = forwardKin(psi(i), l1(i),l2(i));

        end 
    end    
         
    p = [px;py;pz];

       
end