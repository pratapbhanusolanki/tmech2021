%% This script plots the result from 

clear all;
close all;
clc;

load('../ExperimentData/CompactData108to117.mat');  %Change the name here to choose a particular set

dataEKF = CompactData(3).summary;    %Configure the experiment number here for EKF run
dataES = CompactData(9).summary;     %%Configure the experiment number here for ES run
time = dataEKF(1).t;
num_iteration = 200;
T = 0.5;
ES_r = 3*ones(200,1);
for(i=1:2)
    subplot(5,2,i);
    hold on;
    plot(time,dataEKF(i).local_angles(1,:),'r');
    plot(time,dataEKF(i).x_hat(:,2)+dataEKF(i).scan_params(:,1),'b-.');
    plot(time,dataES(i).local_angles(1,:),'k--');
    set(gca,'xticklabel',{[]})
    if(i==2)
    legend('$x_2+\beta$-EKF State','$\hat{x}_2+\beta$-EKF Estimate','$x_2$-ES State','Interpreter','Latex')
    end
    %xlabel('Time') % x-axis label
    if(i==1)
    ylabel(' $x_2 (^\circ)$','Interpreter','Latex');
    title('Rover Data');
    else 
        title('Elevator Data')
    end
    %ylim([-0,0.6]);
    xlim([0,num_iteration*T]);

    subplot(4,2,2+i);
    hold on;
    plot(time,dataEKF(i).local_angles(2,:),'r');
    plot(time,dataEKF(i).x_hat(:,3)+dataEKF(i).scan_params(:,2),'b-.');
    plot(time,dataES(i).local_angles(2,:),'--k');
    set(gca,'xticklabel',{[]})
    %xlabel('Time') % x-axis label
    if(i==1)
    ylabel(' $x_3 (^\circ)$','Interpreter','Latex');
    end
    if(i==2)
    legend('$x_3+\alpha$-EKF State','$\hat{x}_3+\alpha$-EKF Estimate','$x_3$-ES State','Interpreter','Latex')
    end
    
%     %legend('Experimental Estimate','Simulation Estimate')
%     %ylim([-10,10]);
%     xlim([0,num_iteration*T]);
%     subplot(5,2,4+i);
%     plot(time,dataEKF(i).x(3,:),'r');
%     hold on;
%     plot(time,dataEKF(i).x_hat(3,:),'b-.');
%      plot(time,dataES(i).x(3,:),'k--');
%     set(gca,'xticklabel',{[]})
%     %xlabel('Time') % x-axis label
%     if(i==1)
%     ylabel('$$\hat{x}_3\left(^\circ\right)$$','Interpreter','Latex');
%     end
%     
%     %legend('Experimental Estimate','Simulation Estimate')
%     %ylim([-10,10]);
%     xlim([0,num_iteration*T]);
    
    
    subplot(4,2,4+i);
    hold on;
    plot(time, dataEKF(i).y,'r');
    hold on;
    plot(time,dataEKF(i).y_hat,'b-.');
    plot(time,dataES(i).y,'k--');
    set(gca,'xticklabel',{[]})
    if(i==2)
    legend('$y$-EKF State','$\hat{y}$-EKF Estimate','$y$-ES State','Interpreter','Latex');
    end
    if(i==1)
    ylabel('y (V)','Interpreter','Latex');
    end
    %ylim([-0,6]);
    xlim([0,num_iteration*T]);
    
    subplot(4,2,6+i);
    hold on;
    EKF_r = dataEKF(i).scan_params(:,6);
    plot(time,EKF_r,'b-');
    plot(time,ES_r,'k--');
    if (i==2)
    legend('$\delta_r$-EKF Scanning Amplitude','$\delta_{ES}$-ES Perturbation Amplitude','Interpreter','Latex')
    end
    
    ylim([0,5]);
    xlim([0,num_iteration*T]);
    %ylim([-20,20]);
    xlabel('Time $(s)$','Interpreter','Latex') % x-axis label
    if(i==1)
    ylabel('$$\delta_r\left(^\circ\right)$$','Interpreter','Latex');
    end
    %ylim([0,10]);
%     figure(ellipse_fig);
%     subplot(1,2,i);
%     kn = num_iteration/5;
%     for(en=1:kn)
%        plot_index = en*5;
%        draw_ellipse3(dataEKF(i).x_hat(2:3,plot_index),dataEKF(i).P(2:3,2:3,plot_index), time(plot_index));
%        hold on;
%     end
%     plot3(time,dataEKF(i).x_hat(2,:),dataEKF(i).x_hat(3,:),'b-');
end