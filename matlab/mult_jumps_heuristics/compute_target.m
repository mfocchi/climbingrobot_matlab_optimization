function [int_target, l_vec, theta_r] = compute_target(lp_vec, theta_vec)

    %N jumps
    N = length(lp_vec);
% 
%     % compute target bold
%     x = 0;
%     y = 0;
%     z = 0;
%     for i=1:N
%         y = y + lp_vec(i) * sin(sum(theta_vec(i:N)) );
%         z = z - lp_vec(i) * cos(sum(theta_vec(i:N)) );
%     end
% 
%     target = [x,y,z];

    l_vec(1) = lp_vec(1);
    theta_r(1) = 0;

    for i =2:N
       l_vec(i) = sqrt( l_vec(i-1)^2 + lp_vec(i)^2 -2 *    l_vec(i-1)*lp_vec(i) *cos(pi - ( sum(theta_vec(1:i-1)) - sum(theta_r(1:i-1))) ) );
       theta_r(i) = acos(  (  l_vec(i-1)^2 + l_vec(i)^2 -lp_vec(i)^2 )/(2* l_vec(i-1)*l_vec(i)));

    end    

    % this is better for double check and for the fact that it computes
    % intermediate targets
    for i=1:N
        abs_angle = sum( theta_vec(1:i)) - sum(theta_r(1:i));
        int_target(:,i) = [0;  l_vec(i)*sin(abs_angle); -l_vec(i)*cos(abs_angle)];
    end
    
    target = int_target(:,N);

    
    % compute_params([1,1,sqrt(2)], [pi/2, pi/4, pi/4]) should give target = [0;2;0] and theta_r1 = pi/4 theta_r3= pi/4
    
    % double check 
    
    
end