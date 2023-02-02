function normal2 = computeNormalRope(r_leg, anchor,  point)

    rope_direction = (anchor - point )/norm(anchor -point);
    alpha_anchor1 = acos(rope_direction(3)); % angle of the point wrt the vertical line passing throught the anchor 
    
    theta = atan2(point(1)+0.0001,point(2)); % added small amount to have normal1 perpendicular to wall  when 0, 0                                    
    normal1= [sin(theta); cos(theta) ; sin(alpha_anchor1)];   %normal due to the point location
      
    alpha_anchor2 = atan2(r_leg, norm(anchor - point));
    
%     now I need to further rotate  of alpha_anchor2 about an -Y axis perpendicular to normal1     

%     with matirx product
%     Ry=@(angle)[cos(angle)  0      sin(angle)
%                 0           1      0
%              -sin(angle) 0      cos(angle)];
%     
%     normal2 =     Ry(-alpha_anchor2)* normal1 ; 
%          
    % without matirx product
    normal2 =[ normal1(1) *cos(alpha_anchor2) -  normal1(3)*sin(alpha_anchor2);  normal1(2);  normal1(3)*cos(alpha_anchor2) + normal1(1)*sin(alpha_anchor2)];
        
end