clear all;
close all;
clc;
addpath('~/GoogleDrive/PhD-MSU/Research/AlignmentOpticalCommunication/UnderwaterBidiSTAC/Experiments')
num_MC = 100;
u = [0,0,0];%[0;0.5;0.5];
global T;
T = 0.5;
global num_iteration;
num_iteration = 200;
num_sample = num_iteration*1/5;
global ind;
%orient_elevator = -[3.3;2.3;0.2];  %define the orientation matrix of the rover.
%orient_rover = [2.7;1.7;0.3];

%speed_range = [0.002;0.005;0.01;0.02;0.05;0.1;0.2;0.5;1.5;2;3;5;10;20;50;100];
speed_range_net = 0.0:0.1:1.3;
speed_range = (1/sqrt(2))*speed_range_net;

scaling_factor = 1;
pos_rover = scaling_factor*[1.5;-1.37;0.25];
pos_elevator = scaling_factor*[-0.8;-1.37;0.25];%[0;y;0];


dummy_node = NodeES([0;0;0], [0;0;1],1,0);

%Generating initial conditions for angles
theta_rover = rand(1,num_MC)*360;
theta_elevator = rand(1,num_MC)*360;
InitialConditionAmplitude = 8;


for n = 1:length(speed_range)
    speed = speed_range(n); 
%     del_y = (0.15*0.01/0.7)*speed;
%     del_theta = (0.14*0.01*pi/(0.7*180*1.7))*speed;   %using d=r theta(where theta is in radian) 
     del_z = (0.01*0.5)*speed;
     del_y = (0.01*0.5)*speed;
    
    for k=1:num_MC
        d = norm(pos_rover-pos_elevator);
        initial_beta_rover = InitialConditionAmplitude*cosd(theta_rover(k));
        initial_alpha_rover = InitialConditionAmplitude*sind(theta_rover(k));
        rot_rover = dummy_node.rotz(initial_beta_rover)*dummy_node.rotz(initial_alpha_rover);

        initial_beta_elevator = InitialConditionAmplitude*cosd(theta_elevator(k));
        initial_alpha_elevator = InitialConditionAmplitude*sind(theta_elevator(k));
        rot_elevator = dummy_node.rotz(initial_beta_elevator)*dummy_node.roty(initial_alpha_elevator);

        
        orient_rover = rot_rover*(pos_elevator-pos_rover);
        orient_elevator = rot_elevator*(pos_rover-pos_elevator);

        roverEKF = NodeEKF(pos_rover, orient_rover,0);          %-----The 3rd element is the scan alternation flag counter 
        elevatorEKF = NodeEKF(pos_elevator, orient_elevator,1);%-----
        roverES = NodeES(pos_rover, orient_rover,3,120);          %-----The 3rd element is the scan alternation flag counter 
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
            
            %storing the data
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
            elevatorEKF.position = elevatorEKF.position + [0;0;del_z];
            roverEKF.position = roverEKF.position + [0;del_y;0];

            elevatorES.position = elevatorES.position + [0;0;del_z];
            roverES.position = roverES.position + [0;del_y;0];
        end
        [yMean_roverEKF,yStd_roverEKF] = roverEKF.history.tracking_performance();
        [yMean_elevatorEKF,yStd_elevatorEKF] = elevatorEKF.history.tracking_performance();
        
        [xMean_roverEKF,xStd_roverEKF] = roverEKF.history.tracking_performanceX();
        [xMean_elevatorEKF,xStd_elevatorEKF] = elevatorEKF.history.tracking_performanceX();

        [yMean_roverES,yStd_roverES] = roverES.history.tracking_performance();
        [yMean_elevatorES,yStd_elevatorES] = elevatorES.history.tracking_performance();

        [xMean_roverES,xStd_roverES] = roverES.history.tracking_performanceX();
        [xMean_elevatorES,xStd_elevatorES] = elevatorES.history.tracking_performanceX();

        %EKF
        [outEKF_mean_k,outEKF_std_k] = combined_mean_and_std(yMean_roverEKF,yMean_elevatorEKF,yStd_roverEKF, yStd_elevatorEKF, num_sample,num_sample);
        outEKF_mean(k) = outEKF_mean_k;
        outEKF_std(k) = outEKF_std_k;

        [xEKF_mean_k,xEKF_std_k] = combined_mean_and_std(xMean_roverEKF,xMean_elevatorEKF,xStd_roverEKF, xStd_elevatorEKF, num_sample,num_sample);
        xEKF_mean(k) = xEKF_mean_k;
        xEKF_std(k) = xEKF_std_k;

        %Extremum Seeking
        [outES_mean_k,outES_std_k] = combined_mean_and_std(yMean_roverES,yMean_elevatorES,yStd_roverES, yStd_elevatorES, num_sample,num_sample);
        outES_mean(k) = outES_mean_k;
        outES_std(k) = outES_std_k;

        [xES_mean_k,xES_std_k] = combined_mean_and_std(xMean_roverES,xMean_elevatorES,xStd_roverES, xStd_elevatorES, num_sample,num_sample);
        xES_mean(k) = xES_mean_k;
        xES_std(k) = xES_std_k;

        disp(['n = ', num2str(n),' k = ', num2str(k)]);
    end
    [yEKF_mean_performance_n,yEKF_std_performance_n] = compute_array_mean_and_std(outEKF_mean,outEKF_std,2*num_sample);
    yEKF_mean_performance(n) = yEKF_mean_performance_n;
    yEKF_std_performance(n) = yEKF_std_performance_n;
    
    [xEKF_mean_performance_n,xEKF_std_performance_n] = compute_array_mean_and_std(xEKF_mean,xEKF_std,2*num_sample);
    xEKF_mean_performance(n) = xEKF_mean_performance_n;
    xEKF_std_performance(n) = xEKF_std_performance_n;

    [yES_mean_performance_n,yES_std_performance_n] = compute_array_mean_and_std(outES_mean,outES_std,2*num_sample);
    yES_mean_performance(n) = yES_mean_performance_n;
    yES_std_performance(n) = yES_std_performance_n;
    
    [xES_mean_performance_n,xES_std_performance_n] = compute_array_mean_and_std(xES_mean,xES_std,2*num_sample);
    xES_mean_performance(n) = xES_mean_performance_n;
    xES_std_performance(n) = xES_std_performance_n;
end
%%
close all;
figure(1)
ax = gca;
ax.FontSize = 14; 
yyaxis left;
h1EKF = errorbar(speed_range_net, yEKF_mean_performance, yEKF_std_performance);
hold on
h1ES = errorbar(speed_range_net, yES_mean_performance, yES_std_performance,'--');
ylabel('Average Intensity $\mathbf{I}$ (V)', 'interpreter','latex', 'FontSize', 20);
%ylim([0, 0.7]);
xlim([0 max(speed_range_net)]);
yyaxis right
h2EKF = errorbar(speed_range_net, xEKF_mean_performance, xEKF_std_performance);
hold on;
h2ES = errorbar(speed_range_net, xES_mean_performance, xES_std_performance,'--');
ylabel('Average error $\mathbf{E}$ (degree)', 'interpreter','latex', 'FontSize', 20);
%ylim([0, max(mean_performanceX) + max(std_performanceX)]);

%set(gca, 'XScale', 'log')   
%set(gca, 'YScale', 'log')
xlabel('Speed factor $\lambda$', 'interpreter','latex', 'FontSize', 20);
%xlim([0 5]);
%legend([h1EKF, h1ES, h2EKF, h2ES], '$\mathbf{I}$ EKF','$\mathbf{I}$ ES', '$\mathbf{E}$ EKF','$\mathbf{E} ES$','interpreter','latex', 'FontSize', 18);

save('SimMultiSpeedData.mat','speed_range_net','yEKF_mean_performance','yEKF_std_performance','xEKF_mean_performance','xEKF_std_performance'...
    ,'yES_mean_performance','yES_std_performance','xES_mean_performance','xES_std_performance');

