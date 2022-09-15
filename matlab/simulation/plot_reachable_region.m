function a = plot_reachable_region(p0, x_vec, y_vec, z_vec, mu, colormap_vector , label, colorbar_limits)
    figure;hold on;
    h = scatter3(x_vec,y_vec,z_vec,150, colormap_vector,'filled');hold on ;
    alpha = 0.5;
    set(h, 'MarkerEdgeAlpha', alpha, 'MarkerFaceAlpha', alpha);    
    % colomap and bar
    colormap(jet);
    a= colorbar('Location','eastoutside');

    if nargin > 7
        caxis(colorbar_limits);
    end

    a.Label.Interpreter = 'latex';
    a.Label.String = label;
    a.FontSize =20;
    %setting for ISO view
    %a.Label.Position(1) = 3;
    %a.Position = a.Position - [.02 0 0 0];


    %a.Position(4) = 0.5*a.Position(4); %reduce thickness works only for
    %horizontal
    % Vertical line
    plot3([0 p0(1)],[0 p0(2)],[0 p0(3)],'k', 'linewidth',4);  
    %anchor     
    plot3(0,0,0,'Marker','*','Color','k','MarkerSize',30);
    % inital point 
    plot3(p0(1), p0(2), p0(3), 'Marker', '.', 'Color','g', 'MarkerSize',80) ;



    min_x = min(x_vec); 
    max_x = max(x_vec);
    min_y = min(y_vec);
    max_y = max(y_vec);
    max_z = 0;
    max_z_cone = p0(3);
    min_z = min(z_vec);

    % half cone
    pcone1 = [0 0 max_z_cone];
    pcone2 = [max_x max_x*mu max_z_cone];
    pcone3 = [max_x max_x*mu min_z];
    pcone4 = [0 0  min_z];
    Xw = [pcone1(1) pcone2(1) pcone3(1) pcone4(1)];
    Yw = [pcone1(2) pcone2(2) pcone3(2) pcone4(2)];
    Zw = [pcone1(3) pcone2(3) pcone3(3) pcone4(3)];
    h(4) = fill3(Xw, Yw, Zw, 'r', 'linewidth',4,'FaceAlpha',.3 , 'EdgeColor','r' );

    % other half cone
    pcone1 = [0 0 max_z_cone];
    pcone2 = [mu*(max_y) -max_y max_z_cone];
    pcone3 = [mu*(max_y) -max_y min_z];
    pcone4 = [0 0  min_z];
    Xw = [pcone1(1) pcone2(1) pcone3(1) pcone4(1)];
    Yw = [pcone1(2) pcone2(2) pcone3(2) pcone4(2)];
    Zw = [pcone1(3) pcone2(3) pcone3(3) pcone4(3)];
    h(5) = fill3(Xw, Yw, Zw, 'r','linewidth',4, 'FaceAlpha',.3 , 'EdgeColor','r' );    



    %     drawing a wall at X = 0 
    p1 = [0 min_y min_z];
    p2 = [0 max_y min_z];
    p3 = [0 max_y max_z];
    p4 = [0 min_y max_z]; 
    Xw = [p1(1) p2(1) p3(1) p4(1)];
    Yw = [p1(2) p2(2) p3(2) p4(2)];
    Zw = [p1(3) p2(3) p3(3) p4(3)];  
    h(3) = fill3(Xw, Yw, Zw, 'b', 'FaceAlpha',.2  );

    set(gca,'XLim',[min_x max_x])
    set(gca,'YLim',[min_y max_y])
    set(gca,'ZLim',[min_z max_z]) 
    set(gca,'fontsize',30)

    grid on;
    xlabel("X" );
    label = ylabel("Y" );
    %setting for ISO view
    %label.Position=label.Position + [-1,0.1,0];
    zlabel("Z" );
    %title("Reachability region");
    axis equal;
    view(-26,83);


end