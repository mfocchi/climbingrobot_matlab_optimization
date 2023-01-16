% compute kinematic simbolically for the simplifie model
syms phi theta l  
Rx=@(angle) [1       0        0
            0       cos(angle) -sin(angle)
            0       sin(angle)  cos(angle)]
    

Ry=@(angle)[cos(angle)  0      sin(angle)
            0           1      0
         -sin(angle) 0      cos(angle)]

Rz=@(angle)[cos(angle)    -sin(angle) 0
            sin(angle)   cos(angle)   0
            0           0             1]
%%%%%%%%%%%%%%%%%%%%%%%
%rotate about Y axis

H0_T_1 = [Ry(pi/2-theta) , [0;0;0]
          zeros(1, 3), 1]      
   
H1_T_b = [ eye(3), [l;0;0]
          zeros(1, 3), 1]      
      
H0_T_b =  simplify(H0_T_1 *H1_T_b)  


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

