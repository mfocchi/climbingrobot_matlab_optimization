function  Et = eval_total_energy(l , theta, thetad, phid)
    global m g
    Et = m*l^2/2*(thetad^2+sin(theta)^2*phid^2) - m*g*l*cos(theta);
end