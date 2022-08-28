function  plot_curve(p, p_constr, p0,pf,  E, plot_energy, color_input)
      
       plot3(p(1,:), p(2,:), p(3,:) ,'Color',color_input ) ;
       
       hold on ;
       plot3(p_constr(1,:), p_constr(2,:), p_constr(3,:) ,'o', 'Color', color_input ) ;
       

        plot3(p0(1), p0(2), p0(3), 'Marker', '.', 'Color','g', 'MarkerSize',60) ;
        plot3(pf(1), pf(2), pf(3), 'Marker', '.', 'Color','r', 'MarkerSize',60) ;
        grid on;
          view(3)
          
        % axis equal
        xlim([-2, 5])    
        ylim([-pf(2) , pf(2)])    
        zlim([-1.3*abs(pf(3)), 3])

        xlabel('X');
        ylabel('Y');
        zlabel('Z');
     
        view(33,63);
        
    if (plot_energy)
        figure
        plot(E)    
        title('Plotting the Energy')
        grid on
        xlabel('time');
        ylabel('Energy');
    end
    
end