function plot_results_force(Tf_vec, path_length, energy, x_vec, final_cost_vec, tol)
global Fun_max mu


mean_vec = x_vec(:,1); 
Fun_vec = x_vec(:, 2); 
Fut_vec = x_vec(:, 3);

figure
subplot(6,1,1)
plot(Tf_vec, path_length);grid on;
ylabel('path_length')


subplot(6,1,2)
plot(Tf_vec, energy) ;grid on;
ylabel('energy')


subplot(6,1,3)
plot(Tf_vec, Fun_vec); hold on;grid on;
plot(Tf_vec, Fun_max*ones(1, length(Tf_vec)),'ro'); 
plot(Tf_vec, 0*ones(1, length(Tf_vec)),'bo'); 
ylabel('normal constraints')


subplot(6,1,4)
plot(Tf_vec, Fut_vec); hold on;grid on;
plot(Tf_vec, mu*Fun_max*ones(1, length(Tf_vec)),'ro'); 
plot(Tf_vec, -mu*Fun_max*ones(1, length(Tf_vec)),'ro'); 
ylabel('tg constraints')


subplot(6,1,5)
plot(Tf_vec, mean_vec,'ro'); hold on;grid on;
ylabel('mean ')

subplot(6,1,6)
plot(Tf_vec, log(final_cost_vec)); hold on;grid on;
plot(Tf_vec, log(tol*ones(1, length(Tf_vec))),'ro'); 
ylabel('cost')



end