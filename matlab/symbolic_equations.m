
%l const
syms l theta(t) phi(t) t g



p = [l* sin(theta)*cos(phi); 
    l* sin(theta)*sin(phi); 
    - l*cos(theta)]
pd = diff(p, t);
pd =subs( pd,  {str2sym( 'diff(theta(t), t)'), str2sym('diff(phi(t), t)')}, {str2sym('thetad'),str2sym('phid')});
pd =subs( pd, {str2sym('theta(t)'),str2sym('phi(t)')}, {str2sym('theta'),str2sym('phi')});


theta = theta(t)
phi = phi(t)
thetad=diff(theta,t)
phid = diff(phi,t)
f =  cos(theta)*sin(theta)*(phid^2)-(g/l)*sin(theta);
g = -2*(cos(theta)/sin(theta))*phid*thetad;

df_dtheta = diff(f, theta)
df_dthetad = diff(f, thetad)
df_dphi = diff(f, phi)
df_dphid = diff(f, phid)


dg_dtheta = diff(g, theta)
dg_dthetad = diff(g, thetad)
dg_dphi = diff(g, phi)
dg_dphid = diff(g, phid)


A = [0                1              0    0 ;
     df_dtheta ,  df_dthetad , df_dphi, df_dphid ;
    0                0                    0            1 ;
     dg_dtheta ,  dg_dthetad , dg_dphi, dg_dphid ]

% l non const 
