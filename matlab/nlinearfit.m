


%
clear all ; close all ; clc
global m l g  w1 w2 w3 p0 pf N time OLD_FORMULATION
m = 1;
l = 4;
g = 9.81;
Tf = 1.0;

w1 = 1 ; % green initial
w2 = 1 ; %red final
w3 = 0.01 ;

N = 10 ;
OLD_FORMULATION = 1 ;

time = linspace(0, Tf, N) ;
theta0 = pi/6 ; %theta0 = 0.523
phi0 = 0 ;
thetaf= 0.8864 ;
phif = 1.5468 ;

p0 = [l*sin(theta0)*cos(phi0); l*sin(theta0)*sin(phi0); -l*cos(theta0)];
pf = [l*sin(thetaf)*cos(phif); l*sin(thetaf)*sin(phif); -l*cos(thetaf)];

x0 = [0.1*ones(1,8), zeros(1,N)] ;
lb = [-1*ones(1,8), zeros(1,N)];
ub = [1*ones(1,8), 10*ones(1,N)];

options = optimoptions('fmincon','Display','iter','Algorithm','sqp');

[x, final_cost] = fmincon(@cost,x0,[],[],[],[],lb,ub,@contraints, options)
slacks = sum(x(9:end))


plot_curve(x, Tf, p0, pf);


function [coste] = cost(x)

    global N l p0 pf w1 w2 w3 time OLD_FORMULATION


    a_10 = x(1);
    a_11 = x(2);
    a_12 = x(3);
    a_13 = x(4);
    a_20 = x(5);
    a_21 = x(6);
    a_22 = x(7);
    a_23 = x(8);

    
    if (OLD_FORMULATION)
        theta0 = a_10 + a_11*time(1) + a_12*time(1).^2 +  a_13*time(1).^3;
        phi0 = a_20 + a_21*time(1) + a_22*time(1).^2 + a_23*time(1).^3;
        p_0 = [l*sin(theta0).*cos(phi0); l*sin(theta0).*sin(phi0); -l*cos(theta0)];

        thetaf = a_10 + a_11*time(end) + a_12*time(end).^2 +  a_13*time(end).^3;
        phif = a_20 + a_21*time(end) + a_22*time(end).^2 + a_23*time(end).^3;
        p_f = [l*sin(thetaf).*cos(phif); l*sin(thetaf).*sin(phif); -l*cos(thetaf)]; 
    else 
        
        phi0 = a_20 + a_21*time(1) + a_22*time(1).^2 + a_23*time(1).^3 ;   
        z0 = a_10 + a_11*time(1) + a_12*time(1).^2 +  a_13*time(1).^3;
        y0 = sqrt(l^2 -z0^2) *  sin(phi0);
        x0 =  sqrt(l^2 -z0^2) *  cos(phi0);
        p_0 = [x0; y0; z0];


        phif = a_20 + a_21*time(end) + a_22*time(end).^2 + a_23*time(end).^3;
        zf = a_10 + a_11*time(end) + a_12*time(end).^2 +  a_13*time(end).^3;
        yf = sqrt(l^2 -z0^2) *  sin(phif);
        xf =  sqrt(l^2 -z0^2) *  cos(phif);
        p_f = [xf; yf; zf];
    
 
    end

    coste = w1 * norm(p_0 - p0) + w2 * norm(p_f -pf)+ w3 * sum(x(9:end));
   
   
end


function [ineq, eq] = contraints(x)

    global  g N time l OLD_FORMULATION

    close all 
    a_10 = x(1);
    a_11 = x(2);
    a_12 = x(3);
    a_13 = x(4);
    a_20 = x(5);
    a_21 = x(6);
    a_22 = x(7);
    a_23 = x(8);
    
    
    for i=2:N-1 
        sigma(i) = x(8+i);
        
        if (OLD_FORMULATION)
            theta = a_10 + a_11*time(i) + a_12*time(i).^2 +  a_13*time(i).^3;
            thetad =  a_11 + 2*a_12*time(i) + 3*a_13*time(i).^2;
            thetadd = 2*a_12 + 6*a_13*time(i);

            phi = a_20 + a_21*time(i) + a_22*time(i).^2 + a_23*time(i).^3;
            phid =  a_21 + 2*a_22*time(i)  + 3*a_23*time(i).^2;
            phidd =   2*a_22 + 6*a_23*time(i);
%             ineq(i) = norm(l*( thetad*thetadd  + 2*sin(theta)*cos(theta)*thetad^2 +...
%                         sin(theta)^2.*thetad*thetadd) + g*sin(theta)) - sigma(i);
            ineq(i) = norm(l*( thetad*thetadd  + sin(theta)*cos(theta)*thetad*phid^2 +...
                        sin(theta)^2*phid*phidd) + g*sin(theta)*thetad) - sigma(i);

                
        else 
         
            z = a_10 + a_11*time(i) + a_12*time(i).^2 +  a_13*time(i).^3;
            zd =  a_11 + 2*a_12*time(i) + 3*a_13*time(i).^2;
            zdd = 2*a_12 + 6*a_13*time(i);
            phid =  a_21 + 2*a_22*time(i)  + 3*a_23*time(i).^2;
            phidd =   2*a_22 + 6*time(i);


            s_theta = sqrt(1 - z^2/l^2);
            c_theta = z/l;
            thetad = zd / sqrt(l^2 - z^2);
            thetadd = zd^2 / nthroot(l^2-z^2,3) + zdd / sqrt(l^2 - z^2);

%             ineq(i) = norm( l* (thetad * thetadd + 2+s_theta*c_theta * phid^2 + s_theta^2*phid*phidd) + g*s_theta) - sigma(i);
             ineq(i) = norm(l*( thetad*thetadd  + s_theta*c_theta*thetad*phid^2 +...
                        s_theta^2*phid*phidd) + g*s_theta*thetad) - sigma(i);
        end


    end

    eq = [];
end