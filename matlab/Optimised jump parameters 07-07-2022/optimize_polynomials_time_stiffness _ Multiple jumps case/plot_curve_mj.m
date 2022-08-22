function  plot_curve_mj(p, p0,pf,  E, plot_energy, color_input)
    
global x_vec0  x_vecf y_vec0  y_vecf z_vec0  z_vecf 



    figure(1)
    set(gcf,'Position',[50 50 640 640]) % Social
    hold on ;
    grid on ;
%     set(gca,'CameraPosition',[10.0101   30.8293   16.2256])
    set(gca,'CameraPosition',[34.0101   5   5.2256])

%        for n = 1:length(p(1,:))
%           cla
%        hold on
   
%     % Vertical line
%     plot3([0 0],[0 0],[0 p(3,end)],'k--')
%     Point fix
       plot3(0,0,0,'Marker','*','Color','k','MarkerSize',10);
    
      % Pendulum trajectory
      plot3(p(1,:), p(2,:), p(3,:),'b')   ; 
%     plot3(p(1,1:n), p(2,1:n), p(3,1:n),'b')   ;   

      % Pendulum rod
%     plot3([0 p(1,n)],[0 p(2,n)],[0 p(3,n)],'r')
      plot3([0 p(1,end)],[0 p(2,end)],[0 p(3,end)],'r')    
      % Pendulum sphere
%     plot3(p(1,n),p(2,n),p(3,n),'Marker','o','Color','k','MarkerFaceColor','r','MarkerSize',10)
      plot3(p(1,end),p(2,end),p(3,end),'Marker','o','Color','k','MarkerFaceColor','r','MarkerSize',10)
       
%          plot3(p(1,:), p(2,:), p(3,:) ,color_input ) ;  
%          hold on ;
%        plot3(p0(1), p0(2), p0(3), 'Marker', '.', 'Color','g', 'MarkerSize',60) ;
%        plot3(pf(1), pf(2), pf(3), 'Marker', '.', 'Color','r', 'MarkerSize',60) ;


     title('Multiple Jumps for variable length')

     plot3(x_vec0(1), y_vec0(1), z_vec0(1), 'Marker', '.', 'Color','g', 'MarkerSize',60) ;
     plot3(x_vecf(1), y_vecf(1), z_vecf(1), 'Marker', '.', 'Color','m', 'MarkerSize',60) ;

%      plot3(x_vec0(2), y_vec0(2), z_vec0(2), 'Marker', '.', 'Color','y', 'MarkerSize',30) ;
      plot3(x_vecf(2), y_vecf(2), z_vecf(2), 'Marker', '.', 'Color','y', 'MarkerSize',60) ;
 
%      plot3(x_vec0(3), y_vec0(3), z_vec0(3), 'Marker', '.', 'Color','y', 'MarkerSize',30) ;
     plot3(x_vecf(3), y_vecf(3), z_vecf(3), 'Marker', '.', 'Color','r', 'MarkerSize',60) ;
%         grid on;
%         view(3)
          
%         
%         xlim([-3, 3])     %%% axis for points chosen in the wall 
%         ylim([-3 3])    
%         zlim([-3 1])

        xlabel('X');
        ylabel('Y');
        zlabel('Z');
       
%         view(33,63);

        frame = getframe(gcf);  
     
       hold off
       
 % plot the Energy curves       
        figure(2)
        hold on
        plot(E)    
        title('Plotting the Energy')
        grid on
        xlabel('time');
        ylabel('Energy');
        hold off

    
end