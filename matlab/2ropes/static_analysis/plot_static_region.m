function a = plot_static_region(x_vec, y_vec, z_vec, mu, colormap_vector , label, colorbar_limits)
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


    p_anchor1 = [0;-5;0];
    p_anchor2 = [0;5;0];
    %anchor 1    
    plot3(p_anchor1(1),p_anchor1(2),p_anchor1(3),'Marker','*','Color','k','MarkerSize',30);
    %anchor2    
    plot3(p_anchor2(1),p_anchor2(2),p_anchor2(3),'Marker','*','Color','k','MarkerSize',30);
    
    min_x= 0;
    max_x=2;
    min_z = -10;
    max_z = 1;
    min_y = -7;
    max_y = 7;
    
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
    %saxis equal;
    view(83,50);


end