clear all;
close all;
clc;

addpath('Dependencies/');
%%%%%%%%%%%%%%%%%%%%%%%%%%%
scaling_factor = 0.5;
pos_rover = scaling_factor*[1.5;1.37;0.5];
pos_elevator = scaling_factor*[-0.8;-1.37;0];%[0;y;0];

dummy = Node();
v = (pos_elevator - pos_rover);
v = v/norm(v);
orient_rover = v + dummy.angles2coords([10,6])-[1;0;0]; %[2.0;1.7;0.5];
orient_elevator = -v + dummy.angles2coords([-7,10])-[1;0;0];  %define the orientation matrix of the rover.
%%%%%%%%%%%%%%%%%%%%%%%%%%%

rover = NodeEKF(pos_rover, orient_rover,0);          %-----The 3rd element is the scan alternation flag counter 
elevator = NodeEKF(pos_elevator, orient_elevator,1);%-----
 
fig = figure('units','normalized','outerposition',[0 0 1 1]);
rover = rover.draw_graphics();
elevator = elevator.draw_graphics();
    
LOS = line3(rover.position,elevator.position,':','r');
axis equal;
% xlim([0,4]);
%ylim([0,5]);
% zlim([2,6]);

%h_plane = rover.draw_plane();

xlabel('x');
ylabel('y');
zlabel('z');

drawnow;
u = [0,0,0];%[0;0.5;0.5];
global T;
T = 0.7;
global num_iteration;
num_iteration = 100;
global ind;
view(-15,15);
%camroll(210);
k=1
for ind = 1:num_iteration
    rover.delete_graphics();
    elevator.delete_graphics();
    delete(LOS);
    LOS = line3(rover.position,elevator.position,':','r');
    
    rover.prev_instance = rover;
    elevator.prev_instance = elevator;
    
    elevator = elevator.run(rover.prev_instance);
    rover = rover.run(elevator.prev_instance);
    
    rover = rover.draw_graphics();
    elevator = elevator.draw_graphics();
    
    drawnow;
    F(k) = getframe(gcf) ;
    k = k+1;
    %data plotting
    bots = [rover,elevator];
    for i=1:2
        bot = bots(i);
        data{i}.x(:,ind) = bot.x;
        data{i}.x_hat(:,ind) = bot.x_hat;
        data{i}.y(ind) = bot.y(1);
        data{i}.y_hat(ind) = bot.y_hat(1);
        data{i}.r(ind) = bot.scan.r;
        data{i}.P(:,:,ind) = bot.P;
    end
%     y = y+del_y;
%     theta = theta+del_theta;
%     elevator.position = [r*cosd(theta);2;r*sind(theta)];
%     rover.position =[0;y;0] ;
end

%plotting
all_plots_fig = figure();
ellipse_fig = figure();
time = T:T:num_iteration*T;

for(i=1:2)
    figure(all_plots_fig);
    subplot(5,2,i);
    hold on;
    plot(time,data{i}.x(1,:),'--r');
    plot(time,data{i}.x_hat(1,:),'b');
    set(gca,'xticklabel',{[]})
    legend('Actual state','EKF Estimate')
    %xlabel('Time') % x-axis label
    if(i==1)
    ylabel('$$\hat{x}_1$$ (V)','Interpreter','Latex');
    end
    ylim([-0,6]);
    xlim([0,num_iteration*T]);

    subplot(5,2,2+i);
    hold on;
    plot(time,data{i}.x(2,:),'--r');
    plot(time,data{i}.x_hat(2,:),'b');
    set(gca,'xticklabel',{[]})
    %xlabel('Time') % x-axis label
    if(i==1)
    ylabel('$$\hat{x}_2\left(^\circ\right)$$','Interpreter','Latex');
    end
    
    %legend('Experimental Estimate','Simulation Estimate')
    ylim([-10,10]);
    xlim([0,num_iteration*T]);
    subplot(5,2,4+i);
    plot(time,data{i}.x(3,:),'--r');
    hold on;
    plot(time,data{i}.x_hat(3,:),'b');
    set(gca,'xticklabel',{[]})
    %xlabel('Time') % x-axis label
    if(i==1)
    ylabel('$$\hat{x}_3\left(^\circ\right)$$','Interpreter','Latex');
    end
    
    %legend('Experimental Estimate','Simulation Estimate')
    ylim([-10,10]);
    xlim([0,num_iteration*T]);
    
    
    subplot(5,2,6+i);
    hold on;
    plot(time, data{i}.y,'b');
    hold on;
    plot(time,data{i}.y_hat,'r');
    set(gca,'xticklabel',{[]})
    legend('Actual Measurement','EKF Estimate')
    if(i==1)
    ylabel('$$y,\hat{y}$$ (V)','Interpreter','Latex');
    end
    ylim([-0,6]);
    xlim([0,num_iteration*T]);
    
    subplot(5,2,8+i);
    hold on;
    plot(time,data{i}.r,'-r');
    ylim([-0,9]);
    xlim([0,num_iteration*T]);
    %ylim([-20,20]);
    xlabel('Time') % x-axis label
    if(i==1)
    ylabel('$$\delta_r\left(^\circ\right)$$','Interpreter','Latex');
    end
    %ylim([0,10]);
    figure(ellipse_fig);
    subplot(1,2,i);
    kn = num_iteration/5;
    for(en=1:kn)
       plot_index = en*5;
       draw_ellipse3(data{i}.x_hat(2:3,plot_index),data{i}.P(2:3,2:3,plot_index), time(plot_index));
       hold on;
    end
    plot3(time,data{i}.x_hat(2,:),data{i}.x_hat(3,:),'b-');
end




% create the video writer with 1 fps
  writerObj = VideoWriter('Videos/bi-directional.mp4');
  writerObj.FrameRate = 5;
  % set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);
