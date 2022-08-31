function  plot_curve(solution, solution_constr, p0,pf,  plot_energy, color_input)
        global time
        % Vertical line
        plot3([0 p0(1)],[0 p0(2)],[0 p0(3)],'k--');   hold on ;
        %anchor     
        plot3(0,0,0,'Marker','*','Color','k','MarkerSize',10);


        % Min-max axis
        min_x = min(min(solution.p(1,:)), pf(1))-3 ; 
        max_x = max(max(solution.p(1,:)), pf(1))+3 ;
        min_y = min(min(solution.p(2,:)), pf(2))-3 ;
        max_y = max(max(solution.p(2,:)), pf(2)) +3 ;
        min_z = min(pf(3,:))-4;
        max_z = 2;


        hold on ; grid on ; axis equal
        set(gca,'CameraPosition',[10   35   10])
        set(gca,'XLim',[min_x max_x])
        set(gca,'YLim',[min_y max_y])
        set(gca,'ZLim',[min_z max_z])      


        %     drawing a wall at X = 0 
        p1 = [0 min_y min_z];
        p2 = [0 max_y min_z];
        p3 = [0 max_y max_z];
        p4 = [0 min_y max_z]; 
        Xw = [p1(1) p2(1) p3(1) p4(1)];
        Yw = [p1(2) p2(2) p3(2) p4(2)];
        Zw = [p1(3) p2(3) p3(3) p4(3)];  
        h(3) = fill3(Xw, Yw, Zw, 'b', 'FaceAlpha',.5  );


       % actual traj 
       plot3(solution.p(1,:), solution.p(2,:), solution.p(3,:) ,'Color',color_input ) ;
       
       % discrete traj
    
       plot3(solution_constr.p(1,:), solution_constr.p(2,:), solution_constr.p(3,:) ,'o', 'Color', color_input ) ;
       

        plot3(p0(1), p0(2), p0(3), 'Marker', '.', 'Color','g', 'MarkerSize',60) ;
        plot3(pf(1), pf(2), pf(3), 'Marker', '.', 'Color','r', 'MarkerSize',60) ;
        grid on;
          view(3)
          
        % axis equal
  
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
     
        view(33,63);
        
    if (plot_energy)
        figure
        plot(solution.time, solution.energy.Etot)    
        title('Plotting the Energy')
        grid on
        xlabel('time');
        ylabel('Energy');
    end
    
end