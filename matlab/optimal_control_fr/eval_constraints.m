function eval_constraints(c, num_constr, constr_tolerance)

  if any(c(1:  num_constr.wall_constraints)>constr_tolerance)
        disp('1- wall constraints violated')
        c(1:  num_constr.wall_constraints)
    end     
    
    retraction_force_constraints_idx = num_constr.wall_constraints;
    
    if any( c(retraction_force_constraints_idx+1:retraction_force_constraints_idx+num_constr.retraction_force_constraints)>constr_tolerance)
        disp('4- retraction force constraints violated')
         c(retraction_force_constraints_idx+1:retraction_force_constraints_idx + num_constr.retraction_force_constraints)
    end  

    force_constraints_idx = retraction_force_constraints_idx + num_constr.retraction_force_constraints;
    
    if any(c(force_constraints_idx+1: force_constraints_idx + num_constr.force_constraints)>constr_tolerance)
        disp('5 -force constraints violated')
        c(force_constraints_idx+1: force_constraints_idx + num_constr.force_constraints)
        solution.Fun
        solution.Fut
    end
    
end