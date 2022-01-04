%%
clear all;
close all;
clc;


figure(1)

idx = 1:11;
load SimMultiSpeedData.mat
h2EKF = errorbar(speed_range_net(idx), xEKF_mean_performance(idx), xEKF_std_performance(idx),'r-');
hold on;
h2ES = errorbar(speed_range_net(idx), xES_mean_performance(idx), xES_std_performance(idx),'b--');
xlim([0 max(speed_range_net(idx))]);
xticks(speed_range_net(idx));
xtickformat('%.1f');
ax = gca;
ax.FontSize = 14; 
legend([h2EKF, h2ES], 'EKF','ES','interpreter','latex', 'FontSize', 24);
%ylim([0, max(mean_performanceX) + max(std_performanceX)]);

ylabel('Mean Error $\textbf{E} (^\circ)$', 'interpreter','latex', 'FontSize', 20);
%set(gca, 'XScale', 'log')   
%set(gca, 'YScale', 'log')
xlabel('Relative speed $v$ (cm/s)', 'Interpreter','latex', 'FontSize', 20);
%xlim([0 5]);
%legend([h1EKF, h1ES, h2EKF, h2ES], '$\mathbf{I}$ EKF','$\mathbf{I}$ ES', '$\mathbf{E}$ EKF','$\mathbf{E} ES$','interpreter','latex', 'FontSize', 18);

