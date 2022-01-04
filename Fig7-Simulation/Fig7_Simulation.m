% %%
% This script performs single simulation runs of the EKF (Extended Kalman Filter) based approach and the ES (Extremum Seeking) approach. 
% 
% The results are plotted in a figure similar to Fig. 7 of the paper. 
% %%



clear all;
close all;
clc;
addpath('Dependencies/');
theta = 0;
r = 2;
y = 0;
del_theta = 0;%0.05; 
del_y = 0;%0.005;

scaling_factor = 1;
pos_rover = scaling_factor*[1.5;-1.37;0.25];
pos_elevator = scaling_factor*[-0.8;-1.37;0.25];%[0;y;0];

dummy = Node();
v = (pos_elevator - pos_rover);
v = v/norm(v);
orient_rover = v + dummy.angles2coords([8,6])-[1;0;0]; %[2.0;1.7;0.5];
orient_elevator = -v + dummy.angles2coords([-7,5])-[1;0;0];  %define the orientation matrix of the rover.


u = [0,0,0];%[0;0.5;0.5];
global T;
T = 0.5;
global num_iteration;
num_iteration = 200;
global ind;

roverEKF = NodeEKF(pos_rover, orient_rover,0);          %-----The 3rd element is the scan alternation flag counter 
elevatorEKF = NodeEKF(pos_elevator, orient_elevator,1);%-----

roverES = NodeES(pos_rover, orient_rover,3,120);          
elevatorES = NodeES(pos_elevator, orient_elevator,3,90);

for ind = 1:num_iteration
    roverEKF.prev_instance = roverEKF;
    elevatorEKF.prev_instance = elevatorEKF;
    
    elevatorEKF = elevatorEKF.run(roverEKF.prev_instance);
    roverEKF = roverEKF.run(elevatorEKF.prev_instance);
    
    roverES.prev_instance = roverES;
    elevatorES.prev_instance = elevatorES;
    
    elevatorES = elevatorES.run(roverES.prev_instance);
    roverES = roverES.run(elevatorES.prev_instance);
    
    %drawnow;
    %data plotting
    botsEKF = [roverEKF,elevatorEKF];
    botsES = [roverES,elevatorES];
    
    for i=1:2
        botEKF = botsEKF(i);
        dataEKF{i}.x(:,ind) = botEKF.x + [0;botEKF.scan.beta;botEKF.scan.alpha];
        dataEKF{i}.x_hat(:,ind) = botEKF.x_hat + [0;botEKF.scan.beta;botEKF.scan.alpha];;
        dataEKF{i}.y(ind) = botEKF.y(1);
        dataEKF{i}.y_hat(ind) = botEKF.y_hat(1);
        dataEKF{i}.r(ind) = botEKF.scan.r;
        dataEKF{i}.P(:,:,ind) = botEKF.P;
        
        botES = botsES(i);
        dataES{i}.x(:,ind) = botES.x+[0;botES.scan.beta;botES.scan.alpha];;
        dataES{i}.y(ind) = botES.y(1);
        dataES{i}.r(ind) = botES.scan.r;
        dataES{i}.beta(ind) = botES.scan.beta;
        dataES{i}.alpha(ind) = botES.scan.alpha;
       
    end

    
    
%     y = y+del_y;
%     theta = theta+del_theta;
%     elevator.position = [r*cosd(theta);2;r*sind(theta)];
%     rover.position =[0;y;0] ;
end
%plotting
all_plots_fig = figure();
%ellipse_fig = figure();
time = T:T:num_iteration*T;

for(i=1:2)
    figure(all_plots_fig);
    subplot(5,2,i);
    hold on;
    plot(time,dataEKF{i}.x(1,:),'r');
    plot(time,dataEKF{i}.x_hat(1,:)/1,'b-.');
    plot(time,dataES{i}.x(1,:),'k--');
   
    set(gca,'xticklabel',{[]})
    
    %xlabel('Time') % x-axis label
    if(i==1)
    ylabel('$$x_1\ \left(^\circ\right)$$','Interpreter','Latex');
    title('Rover Data');
    legend('$x_1$-EKF State','$\hat{x}_1$-EKF Estimate','$x_1$-ES State','Interpreter','Latex')
    else 
        title('Elevator Data')
    end
    %ylim([-0,0.6]);
    xlim([0,num_iteration*T]);

    subplot(5,2,2+i);
    hold on;
    plot(time,dataEKF{i}.x(2,:),'r');
    plot(time,dataEKF{i}.x_hat(2,:),'b-.');
    plot(time,dataES{i}.x(2,:),'--k');
   
    set(gca,'xticklabel',{[]})
    %xlabel('Time') % x-axis label
    if(i==1)
    ylabel('$$x_2\ \left(^\circ\right)$$','Interpreter','Latex');
    legend('$x_2$-EKF State','$\hat{x}_2$-EKF Estimate','$x_2$-ES State','Interpreter','Latex');
    end
    
    %legend('Experimental Estimate','Simulation Estimate')
    %ylim([-10,10]);
    xlim([0,num_iteration*T]);
    subplot(5,2,4+i);
    plot(time,dataEKF{i}.x(3,:),'r');
    hold on;
    plot(time,dataEKF{i}.x_hat(3,:),'b-.');
    plot(time,dataES{i}.x(3,:),'k--');
   
    set(gca,'xticklabel',{[]})
    %xlabel('Time') % x-axis label
    if(i==1)
    ylabel('$$x_3\ \left(^\circ\right)$$','Interpreter','Latex');
    legend('$x_3$-EKF State','$\hat{x}_3$-EKF Estimate','$x_3$-ES State','Interpreter','Latex');
    end
    
    %legend('Experimental Estimate','Simulation Estimate')
    %ylim([-10,10]);
    xlim([0,num_iteration*T]);
    
    
    subplot(5,2,6+i);
    hold on;
    plot(time, dataEKF{i}.y,'r');
    hold on;
    plot(time,dataEKF{i}.y_hat,'b-.');
    plot(time,dataES{i}.y,'k--');
    set(gca,'xticklabel',{[]})
    
    if(i==1)
        legend('$y$-EKF Output','$\hat{y}$-EKF Estimate','$y$-ES Output','Interpreter','Latex')
    ylabel('$y$ (V)','Interpreter','Latex');
    end
    %ylim([-0,6]);
    xlim([0,num_iteration*T]);
    
    subplot(5,2,8+i);
    hold on;
    plot(time,dataEKF{i}.r,'b-');
    plot(time,dataES{i}.r,'k--');
    
    
    ylim([0,5]);
    xlim([0,num_iteration*T]);
    %ylim([-20,20]);TMETMECHT
    xlabel('Time $(s)$','Interpreter','Latex') % x-axis label
    if(i==1)
       legend('$\delta_r$-EKF','$\delta_{ES}$-ES','Interpreter','Latex')
    ylabel('$$\delta_r, \delta_{ES}\  \left(^\circ\right)$$','Interpreter','Latex');
    end
end


