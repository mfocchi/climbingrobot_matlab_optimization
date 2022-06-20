function [target, l_vec] = compute_params(lp_vec, theta_vec)

    %N jumps
    N = length(lp_vec);

    % compute target
    x = 0;
    y = 0;
    z = 0;
    for i=1:N
        y = y + lp_vec(i) * sin(sum(theta_vec(i:N)) );
        z = z - lp_vec(i) * cos(sum(theta_vec(i:N)) );
    end

    target = [x,y,z];

    l_vec(1) = lp_vec(1);
    theta_r(1) = 0;

    for i =2:N
       l_vec(i) = sqrt( l_vec(i-1)^2 + lp_vec(i)^2 -2 *    l_vec(i-1)*lp_vec(i) *cos(pi - ( sum(theta_vec(1:i-1)) - sum(theta_r(1:i-1))) ) )
       theta_r(i) = acos(  (  l_vec(i-1)^2 + l_vec(i)^2 -lp_vec(i)^2 )/(2* l_vec(i-1)*l_vec(i)));

    end    

end