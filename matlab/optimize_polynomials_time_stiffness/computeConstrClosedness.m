function viol = computeConstraintClosedness(threshold,  value, lower, upper)

   
   % residual_lower = value - lower;      
   % residual_upper = upper - value
   % viol =  1/(1 + (1/influence)*residual_lower^2) + 1/(1 + (1/influence)*residual_upper^2);

    residual_upper_th = upper - threshold/2*(upper - lower);
    sharpness = 3/(upper -residual_upper_th);
    viol_up = 0.5 + 0.5*tanh(sharpness*(value - residual_upper_th));
    
    residual_lower_th = lower + threshold/2*(upper - lower);
    viol_down= 0.5 + 0.5*tanh(sharpness*(residual_lower_th - value ));
    
    viol = viol_up + viol_down;
end

