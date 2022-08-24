function [x_vec,  t_vec] = dynamics_autonomous(x0, dt,n_steps, K)
    %verify is a column vector
    x0 = x0(:);
    t_ = 0.;
    x_ = x0;
    x_vec = [];
    t_vec = [];
    for i=1:n_steps
        x_ = x_ + dt* dynamics_autonomous(x_, K);
        t_ = t_ + dt;
        x_vec = [x_vec x_];
        t_vec = [t_vec t_];
    end

end