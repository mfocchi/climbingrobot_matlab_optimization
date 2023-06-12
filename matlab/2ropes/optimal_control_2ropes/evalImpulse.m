function [fleg] = evalImpulse(t, Fleg)
global    T_th 

    if (t <= T_th)
        fleg = Fleg(:);% make sure is a column vector
    else
        fleg = [0;0;0];
    end
  
end