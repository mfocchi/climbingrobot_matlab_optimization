function [ineq, eq] = constraints(x, l, DER_ENERGY_CONSTRAINT)

global  g N   m num_params

% ineq are <= 0
Tf = x(1);
time =  linspace(0, Tf, N) ;
a_10 = x(2);
a_11 = x(3);
a_12 = x(4);
a_13 = x(5);
a_20 = x(6);
a_21 = x(7);
a_22 = x(8);
a_23 = x(9);

% parametrizzation with sin theta sing phi
arg1 = a_10 + a_11*time + a_12*time.^2 +  a_13*time.^3;
arg1d = a_11 + 2*a_12*time + 3*a_13*time.^2;
s_theta = arg1;
c_theta = sqrt(1 -  s_theta.^2);
arg2 = a_20 + a_21*time + a_22*time.^2 + a_23*time.^3;
arg2d =  a_21 + 2*a_22*time  + 3*a_23*time.^2;
s_phi = arg2;
c_phi = sqrt(1 -  s_phi.^2);
thetad2 = 1./(1-arg1).*arg1d.^2;
phid2 = 1./(1-arg2).*arg2d.^2;


    
    
for i=1:N-1     
    
    E(i) = m*l^2/2*(thetad2(i)+s_theta(i)^2*phid2(i) ) - m*g*l*c_theta(i);
    sigma(i) = x(num_params+i);        
    if (i>=2)
        ineq(i) = abs(E(i) - E(i-1)) - sigma(i);
    end

end

%impose in range -1,1 only extremes of the range
ineq = [ineq (s_theta(1)-1)];
ineq = [ineq (-s_theta(1)-1)];
ineq = [ineq (s_phi(1)-1)];
ineq = [ineq (-s_phi(1)-1)];

ineq = [ineq (s_theta(end)-1)];
ineq = [ineq (-s_theta(end)-1)];
ineq = [ineq (s_phi(end)-1)];
ineq = [ineq (-s_phi(end)-1)];

% discriminant of derivative
% delta1 = 4*a_12^2 -12*a_13*a_11;
% delta2 = 4*a_22^2 -12*a_23*a_21;
% 
% % discriminant position I have two more point to ensure they are in the
% % range otherwise sin is regular
% 
% int_solution_exist1 =  (delta1>0 ) && (abs(a_23)>0.000001);
% %find  the time instants where this happens
% t1a =  ( -4*a_12^2 +sqrt(delta1)) /(6*a_13);
% t1b =  ( -4*a_12^2 -sqrt(delta1)) /(6*a_13);
% if int_solution_exist1 && (t1a>0)         
%     arg_t1a = a_10 + a_11*t1a + a_12*t1a^2 +  a_13*t1a^3
%     ineq = [ineq (arg_t1a-1)];
%     ineq = [ineq (-arg_t1a-1)];
% else
%      ineq = [ineq 0];
%      ineq = [ineq 0];
% end
% 
% if int_solution_exist1 && (t1b>0)         
%     arg_t1b = a_10 + a_11*t1b + a_12*t1b.^2 +  a_13*t1b.^3
%     ineq = [ineq (arg_t1b-1)];
%     ineq = [ineq (-arg_t1b-1)];
% else
%     ineq = [ineq 0];
%     ineq = [ineq 0];
% end   
% 
% 
% int_solution_exist2 = (delta2 >0) && (abs(a_23)>0.000001);
% %find  the time instants where this happens
% t2a =  ( -4*a_22^2 +sqrt(delta2)) /(6*a_23);
% t2b =  ( -4*a_22^2 -sqrt(delta2)) /(6*a_23);
% if int_solution_exist2 && (t2a>0)         
%     arg_t2a = a_10 + a_11*t2a + a_12*t2a.^2 +  a_13*t2a.^3;
%     ineq = [ineq (arg_t2a-1)];
%     ineq = [ineq (-arg_t2a-1)];
% else
%      ineq = [ineq 0];
%      ineq = [ineq 0];
% end   
% 
% if int_solution_exist2 && (t2b>0)
%     arg_t2b = a_10 + a_11*t2b + a_12*t2b.^2 +  a_13*t2b.^3;
%     ineq = [ineq (arg_t2b-1)];
%     ineq = [ineq (-arg_t2b-1)];
% else
%     ineq = [ineq 0];
%     ineq = [ineq 0];
% end   



eq = [];
end