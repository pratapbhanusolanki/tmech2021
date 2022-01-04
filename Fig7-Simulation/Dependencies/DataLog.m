classdef DataLog
   properties
      %Global pos variables
      position;
      orientation;
      
      %States
      x
      
      %Scanning parameters
      bias;    %Scanning angle
      phi;     %Scanning angle interval
      scan_radius; % scanning amplitude
      r;
      measurements;
      
      %ekf parameters
      P_prior;
      P;
      x_hat_prior;
      x_hat;
      x_hat_I;
      difference;
      y;
      y_hat;
      
      %controller
      u;
      P_eig;
      alpha;
      beta;
      psi;
      
      net_angle;
   end
   
   methods
      function obj = DataLog()
          global num_iteration;
          global ind;
          
          %Actual States
          x = zeros(3,num_iteration);
          
          %World Pos
          obj.position = zeros(3,num_iteration);
          obj.orientation = zeros(3,num_iteration);
          
          %Scanning parameters
          obj.bias = zeros(1,num_iteration);    %Scanning angle
          obj.phi = zeros(1,num_iteration);     %Scanning angle interval
          obj.scan_radius = zeros(1,num_iteration); % scanning amplitude
          obj.alpha = zeros(1,num_iteration); % scanning amplitude
          obj.beta = zeros(1,num_iteration); % scanning amplitude
          obj.psi = zeros(1,num_iteration); % scanning amplitude
          
          %ekf parameters
          obj.P_prior = zeros(3,3,num_iteration);
          obj.P = zeros(3,3,num_iteration);
          obj.x_hat_prior = zeros(3,num_iteration);
          obj.x_hat = zeros(3,num_iteration);
          obj.y = zeros(1,num_iteration);
          
          %controller
          obj.u = zeros(2,num_iteration);

      end
      
      function [yMean,yStd] = tracking_performance(obj)
          global num_iteration;
          n_min = num_iteration*4/5;
          yMean = mean(obj.y(n_min+1:end));
          yStd = std(obj.y(n_min+1:end));
      end
      
      function [yMean,yStd] = tracking_performanceX(obj)
          global num_iteration;
          n_min = num_iteration*4/5;
          yMean = mean(obj.net_angle(n_min+1:end));
          yStd = std(obj.net_angle(n_min+1:end));
      end
      
      function y = tracking_performanceXold(obj)
          global num_iteration;
          n_min = num_iteration/2;
          
          x = obj.x(2:3,:);
          x_hat = obj.x_hat(2:3,:);
          x_ab=abs(x);
          y = sum(sqrt(x_ab(1,:).^2 + x_ab(2,:).^2))/n_min;
      end
   end
end