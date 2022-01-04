%% Run this script to plot from the saved Data. 

close all;
clear all;
load SimMultidistanceData.mat
clc;

addpath('../Fig7-Simulation/Dependencies/');

dummy_node = NodeES([0;0;0], [0;0;1],1,0);
c = dummy_node.c;
Ad = dummy_node.Ad;
for n=1:length(distances)
    d = distances(n);
    LOS_model(n) = Ad*g_comp([0,0])*h_comp([0,0])*exp(c*d)/d^2;
end



figure(1)
ax = gca;
ax.FontSize = 14; 
subplot(2,1,1)
h1EKF = errorbar(distances, yEKF_mean_performance, yEKF_std_performance);
hold on
h1ES = errorbar(distances, yES_mean_performance, yES_std_performance,'--');
ylabel('Average Intensity $\mathbf{I}$ (V)', 'interpreter','latex', 'FontSize', 20);
h1LOS = plot(distances, LOS_model,'k-.','LineWidth',1);
%ylim([0, 0.7]);
xlim([0 max(distances)]);
set(gca,'xticklabel',{[]})
ax = gca;
ax.FontSize = 14; 

legend([h1LOS,h1EKF, h1ES],'LOS', 'EKF','ES','interpreter','latex', 'FontSize', 24);


subplot(2,1,2)
h2EKF = errorbar(distances, xEKF_mean_performance, xEKF_std_performance);
hold on;
h2ES = errorbar(distances, xES_mean_performance, xES_std_performance,'--');
ylabel('Average error $\mathbf{E}$ (degree)', 'interpreter','latex', 'FontSize', 20);
%ylim([0, max(mean_performanceX) + max(std_performanceX)]);

%set(gca, 'XScale', 'log')   
%set(gca, 'YScale', 'log')
xlabel('Distance $d$ (m)', 'interpreter','latex', 'FontSize', 20);
legend([h2EKF, h2ES], 'EKF','ES','interpreter','latex', 'FontSize', 24);
%xlim([0 5]);
xlim([0 max(distances)]);
ax = gca;
ax.FontSize = 14; 