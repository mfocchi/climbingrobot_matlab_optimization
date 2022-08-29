function [x_, t_, x_vec,  t_vec] = integrate_dynamics(x0, t0, dt,n_steps, K)
    %verify is a column vector
    x0 = x0(:);
    t_ = t0;
    x_ = x0;
    x_vec = x0;
    t_vec = 0;
    
    for i=1:n_steps-1
        x_ = x_ + dt* dynamics_autonomous(x_, K);
        t_ = t_ + dt;
        x_vec = [x_vec x_];
        t_vec = [t_vec t_];
    end

end