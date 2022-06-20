







load lookuptables.mat




eval_cost()
eval_constraints()



plot_surf("feasible", l_range_dense, thetaf_range_dense, lthetaf_vector(1,:) ,   lthetaf_vector(2,:),    feasible);
plot_surf("conv",     l_range_dense, thetaf_range_dense, lthetaf_vector(1,:) ,   lthetaf_vector(2,:),    converged);
plot_surf("initial kin energy",l_range_dense, thetaf_range_dense, lthetaf_vector(1,:) ,   lthetaf_vector(2,:),    initial_kin_energy);
plot_surf("final kin energy",l_range_dense, thetaf_range_dense, lthetaf_vector(1,:) ,   lthetaf_vector(2,:),    final_kin_energy);
