close all

baseline = 0.4;
wall_clearance = 0.5;
mu = 0.8;
rope_length = 5;

max_feasible_pos_baseline=[];
baseline_vec = [];

% plot according to base_line
for rope_length=2:0.5:10
    for baseline=0.3:0.02:1.2    
        pos = getMaxFeasiblePoint(baseline, mu, wall_clearance, rope_length); 
        max_feasible_pos_baseline = [max_feasible_pos_baseline pos];
        baseline_vec = [baseline_vec baseline];
    end
end
%old not projected
%plot_static_region(max_feasible_pos_baseline(1,:), max_feasible_pos_baseline(2,:), max_feasible_pos_baseline(3,:), mu, baseline_vec, 'baseline [m]', [5, 45]);    
plot_static_region2D(max_feasible_pos_baseline(1,:), max_feasible_pos_baseline(2,:), max_feasible_pos_baseline(3,:), mu, baseline_vec, 'baseline [m]', [5, 45]);
    
%save the plot
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [25 20]);
set(gcf, 'PaperPosition', [0 0 25 20]);
print(gcf, '-dpdf',strcat('static_analysis_baseline.pdf'),'-painters')

max_feasible_pos_clearance=[];
wall_clearance_vec = [];
% plot according to wall_clearance
for rope_length=2:0.5:10
    for wall_clearance=0.3:0.02:1.2    
        pos = getMaxFeasiblePoint(baseline, mu, wall_clearance, rope_length); 
        max_feasible_pos_clearance = [max_feasible_pos_clearance pos];
        wall_clearance_vec = [wall_clearance_vec wall_clearance];
    end
end    
%old not projected
%plot_static_region(max_feasible_pos_clearance(1,:), max_feasible_pos_clearance(2,:), max_feasible_pos_clearance(3,:), mu, wall_clearance_vec, 'wall clearance [m]', [5, 45]);
plot_static_region2D(max_feasible_pos_clearance(1,:), max_feasible_pos_clearance(2,:), max_feasible_pos_clearance(3,:), mu, wall_clearance_vec, 'clearance [m]', [5, 45]);

%save the plot
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [25 20]);
set(gcf, 'PaperPosition', [0 0 25 20]);
print(gcf, '-dpdf',strcat('static_analysis_wall_clearance.pdf'),'-painters')

