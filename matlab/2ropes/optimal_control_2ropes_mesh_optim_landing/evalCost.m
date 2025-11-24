function val = evalCost(z_query, y_query, params)
    

    costEvalFcn = @(z_query, y_query) interp2(params.cost_z, params.cost_y,  params.cost_x, z_query, y_query, 'linear', 0);
    val = costEvalFcn(z_query, y_query);
end