function normal2cone = computeNormalCone(r_leg, point)



phiCone = 3;
thetaCone = atan2(point(1),point(2));                                    
normal2cone= [sin(thetaCone); cos(thetaCone) ; sin(pi-phiCone)];    







end