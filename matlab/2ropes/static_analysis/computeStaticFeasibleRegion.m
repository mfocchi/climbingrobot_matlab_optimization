

baseline = 0.4;
wall_clearance = 0.5;
mu = 0.8;
rope_length = 5;

pos_vec_baseline=[];
baseline_vec = [];

% plot according to base_line
for rope_length=5:1:10
    for baseline=0.3:0.05:1.2    
        pos = getMaxFeasiblePoint(baseline, mu, wall_clearance, rope_length); 
        pos_vec_baseline = [pos_vec_baseline pos];
        baseline_vec = [baseline_vec baseline];
    end
end
    
plot_static_region(pos_vec_baseline(1,:), pos_vec_baseline(2,:), pos_vec_baseline(3,:), mu, baseline_vec, 'baseline [m]', [5, 45]);


pos_vec_wall_clearance=[];
wall_clearance_vec = [];
% plot according to wall_clearance
for rope_length=5:1:10
    for wall_clearance=0.3:0.05:1.2    
        pos = getMaxFeasiblePoint(baseline, mu, wall_clearance, rope_length); 
        pos_vec_wall_clearance = [pos_vec_wall_clearance pos];
        wall_clearance_vec = [wall_clearance_vec wall_clearance];
    end
end    

plot_static_region(pos_vec_wall_clearance(1,:), pos_vec_wall_clearance(2,:), pos_vec_wall_clearance(3,:), mu, wall_clearance_vec, 'wall clearance [m]', [5, 45]);


