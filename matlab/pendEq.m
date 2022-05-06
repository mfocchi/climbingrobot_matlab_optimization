global dmin; %Minimum distance from the mointain wall
dmin = 0.5;
global dtol; %Tolerance for binding reaction of the wall
dtol = 1e-05;

global delta_duration;
delta_duration = 1e-3;
global Fu0;
Fu0 = 1000;
global Ft0;
Ft0 = 100;
m = 10;
global Tsim;
Tsim = 10;
tspan = [0:0.0001: Tsim];
x0 = [asin(dmin/5); 0; 0; 0; 5; 0];
[t,x] = ode23(@(t,x) diffEq(t,x,m,@Fr,@Fu, @Ft), tspan, x0);
X = x(:,5).*cos(x(:,3)).*sin(x(:,1));
Y = x(:,5).*sin(x(:,3)).*sin(x(:,1));
Z = -x(:,5).*cos(x(:,1));
plot3(X,Y,Z)

%Frist component brake. Second component force.
function [fr] = Fr(t)
global Tsim;
if ((logical(t >= 0.10*Tsim)& logical(t <= 0.15*Tsim))),
  %  fprintf('t = %f. Brakes off\n',t);
    fr=[0,0];
else
   % fprintf('t = %f. Brakes on\n',t);
    fr = [1,0];
end
end

function [fu] = Fu(t)
    global delta_duration;
    global Fu0;
    if (t <= delta_duration)
        fu = Fu0* 1/delta_duration;
    else
        fu = 0;
    end
end

function [ft] = Ft(t)
    global delta_duration;
    global Ft0;
    if (t <= delta_duration)
        ft = Ft0* 1/delta_duration;
    else
        ft = 0;
    end
end

function [dxdt] = diffEq(t,x, m, Fr, Fun ,Fut)
global dmin;
global dtol;
%x = [theta, dottheta, phi, dotphi, l, dotl]
g = 9.8;
theta = x(1);
dtheta = x(2);
phi = x(3);
dphi = x(4);
l = x(5);
dl = x(6);
thmin = asin(dmin/l);
thtol = asin(dtol/l);
FR = Fr(t);
if (theta > thmin + thtol)
  %  fprintf('Flying\n');
    %Flight dynamics
    ddtheta = -2*dtheta*dl/l + cos(theta)*sin(theta)*(dphi^2)- (g/l)*sin(theta);
    ddphi = -2*(cos(theta)/sin(theta))*dphi*dtheta-(2/l)*dphi*dl;
    ddl = l*(dphi^2)+l*(sin(theta)^2)*dphi+g*cos(theta)+(1/m)*FR(2);
else
    if (Fun(t) > 0)
   %     fprintf('Taking off\n');
        %Take off dynamicx
        ddtheta = -2*dtheta*dl/l + cos(theta)*sin(theta)*(dphi^2)+Fun(t)/(m*l);
        ddphi = -2*(cos(theta)/sin(theta))*dphi*dtheta-(2/l)*dphi*dl+ Fut(t)/(m*l*sin(theta));
        ddl = l*(dphi^2)+l*(sin(theta)^2)*dphi+g*cos(theta)+(1/m)*FR(2);
    else
        
        %Parking
    %    fprintf('Parking\n');

        ddtheta = 0;
        dtheta = 0;
        ddphi = 0;
        dphi = 0;
        ddl = 0;
        dl = 0;
    end
end
%Brake on?
if FR(1) > 0.99,
    ddl = 0;
    dl =0;
    fprintf('Brakes on\n');
else
    fprintf('Brakes off\n');
end
dxdt = [dtheta; ddtheta; dphi; ddphi; dl; ddl];
if(FR(1)<0.99)
display(dxdt)
end
end