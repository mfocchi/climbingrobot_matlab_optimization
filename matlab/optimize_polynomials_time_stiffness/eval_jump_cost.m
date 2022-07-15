function cost = eval_jump_cost( weights, intEkin, final_kin_energy, Fun_max, opt_Fun, Fr_max, Fr, number_of_converged_solutions)

    if isnan(number_of_converged_solutions)

        cost = inf;
    else

        cost_fun = computeConstrClosedness(0.1, opt_Fun, 0.0, Fun_max);
        cost_fr = computeConstrClosedness(0.1, Fr, -Fr_max, 0);
  
        cost = weights(1)*intEkin + weights(2)*final_kin_energy + weights(3)*cost_fun + weights(4)*cost_fr;
    end

end