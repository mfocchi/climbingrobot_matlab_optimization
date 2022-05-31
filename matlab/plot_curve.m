function plot_curve(x, Tf, p0, pf)
    global l m g OLD_FORMULATION 
    %eval trajectory
    a_10 = x(1);
    a_11 = x(2);
    a_12 = x(3);
    a_13 = x(4);
    a_20 = x(5);
    a_21 = x(6);
    a_22 = x(7);
    a_23 = x(8);
    t = linspace(0, Tf, 1000);
    
    
    if (OLD_FORMULATION)
        theta = a_10 + a_11*t + a_12*t.^2 +  a_13*t.^3;
        thetad =  a_11 + 2*a_12*t + 3*a_13*t.^2;
        thetadd = 2*a_12 + 6*a_13*t;

        phi = a_20 + a_21*t + a_22*t.^2 + a_23*t.^3;
        phid =  a_21 + 2*a_22*t  + 3*a_23*t.^2;
        phidd =   2*a_22 + 6*a_23*t;

        p = [l*sin(theta).*cos(phi); l*sin(theta).*sin(phi); -l*cos(theta)];    


    else

        z = a_10 + a_11*t + a_12*t.^2 +  a_13*t.^3;
        zd =  a_11 + 2*a_12*t + 3*a_13*t.^2;
        zdd = 2*a_12 + 6*a_13*t;

        phi = a_20 + a_21*t + a_22*t.^2 + a_23*t.^3;
        phid =  a_21 + 2*a_22*t  + 3*a_23*t.^2;
        phidd =   2*a_22 + 6*a_23*t;

        p = [sqrt(l^2 - z.^2).*cos(phi) ; sqrt(l^2 - z.^2).*sin(phi);   z];   
    end

    plot3(p(1,:), p(2,:), p(3,:))   ;   
    hold on ;
     plot3(p0(1), p0(2), p0(3), 'Marker', '*', 'Color','g', 'MarkerSize',10) ;
     plot3(pf(1), pf(2), pf(3), 'Marker', '*', 'Color','r', 'MarkerSize',10) ;
    grid on;
    
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    % Calculating and ploting the Energy from the new fit: theta, thetad and phid

%Energy for OLD_FORMULATION
    E = ((m*l^2/2).*(thetad.^2 + sin(theta).^2 .*phid.^2)) - m*g*l.*cos(theta); 
    figure(2)
    plot(t,E(1,:))
    title('Ploting the Energy')
    grid on
    xlabel('time');
    ylabel('Energy');

% %Energy else
% E = m*(l^2).*(zd.^2))./(2*(l^2-z.^2)) + (m*(l^2-z.^2).*phid.^2)/2 + m*g.*z ;
%     figure(2)
%     plot(t,E(1,:))
%     title('Ploting the Energy')
%     grid on
%     xlabel('time');
%     ylabel('Energy');


    
end