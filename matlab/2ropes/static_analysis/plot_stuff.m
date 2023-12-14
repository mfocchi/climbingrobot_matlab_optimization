function plot_stuff(p_anchor1, p_anchor2, p_base, w_pf1, w_pf2, fr1, fr2, w_pr1, w_pr2, margin)
    min_z = -7;   
    max_z = 1;
    min_y = p_anchor1(2)-2;
    max_y = p_anchor2(2)+2;

    % for paper 
    %clf(gcf)
    set(0, 'DefaultAxesBox', 'on');
    set(0, 'DefaultTextFontSize', 30);
    set(0, 'DefaultAxesFontSize', 30);
    set(0, 'DefaultUicontrolFontSize', 30);



    % plot world
    plot3(0,0,0,'.k', 'MarkerSize',40);grid on;hold on;
    % plot anchors
    plot3(p_anchor1(1),p_anchor1(2),p_anchor1(3),'.m', 'MarkerSize',60);grid on;hold on;
    plot3(p_anchor2(1),p_anchor2(2),p_anchor2(3),'.y', 'MarkerSize',60);grid on;hold on;
    % plot feet pos
    plot3(w_pf1(1), w_pf1(2),w_pf1(3),'.b', 'MarkerSize',40);hold on;
    plot3(w_pf2(1), w_pf2(2),w_pf2(3),'.b', 'MarkerSize',40);hold on;
    % plot base pos
    plot3(p_base(1), p_base(2),p_base(3),'.g', 'MarkerSize',40);hold on;
    xlabel('$X$','interpreter', 'latex');
    ylabel('$Y$','interpreter', 'latex');
    zlabel('$Z$','interpreter', 'latex');


    %     drawing a wall at X = 0
    p1 = [0 min_y min_z];
    p2 = [0 max_y min_z];
    p3 = [0 max_y max_z];
    p4 = [0 min_y max_z];
    Xw = [p1(1) p2(1) p3(1) p4(1)];
    Yw = [p1(2) p2(2) p3(2) p4(2)];
    Zw = [p1(3) p2(3) p3(3) p4(3)];
    h = fill3(Xw, Yw, Zw, 'b', 'FaceAlpha',.5  ); 
    fill3([0.001,0.001,0.001,0.001] , [p_anchor1(2)-margin,p_anchor1(2)-margin , p_anchor2(2)+margin,  p_anchor2(2)+margin], [-7,0, 0, -7],'r', 'FaceAlpha',.5, 'EdgeColor','none'  );

    
    %plot rope forces
arrow3d_points(p_base,p_base+w_pr1 * fr1*force_scale,'color','m');grid on;hold on;
arrow3d_points(p_base,p_base+w_pr2 * fr2*force_scale,'color','y');grid on;hold on;


%plot feet forces
if normal1_viol || F1_viol
    arrow3d_points(w_pf1,w_pf1+fl1*10*force_scale,'color','k');grid on;hold on;
else
    arrow3d_points(w_pf1,w_pf1+fl1*10*force_scale,'color','b');grid on;hold on;
end

if normal2_viol || F2_viol
   arrow3d_points(w_pf2,w_pf2+fl2*10*force_scale,'color','k');grid on;hold on;

else
   arrow3d_points(w_pf2,w_pf2+fl2*10*force_scale,'color','b');grid on;hold on;
end


%plot world reference frame
Tt = [eye(3), [0;0;0];
    zeros(1,3) 1];
tt = hgtransform('Matrix', Tt);
ht = triad('Parent',tt, 'linewidth', 6);


%plot base reference frame
Tt = [wRb, p_base;
    zeros(1,3) 1];
tt = hgtransform('Matrix', Tt);
%ht = triad('Parent',tt, 'linewidth', 6);


%fundamental to see perpedicuolaity
xlim([0, 2])
ylim([min_y,max_y])
zlim([min_z,max_z])
axis equal
view(60,30)
    
end