 classdef NodeEKF
   properties
      %Global pos variables
      position;
      orientation;
      orientation_angles
      local_vectors;
      
      %States
      x
      
      %model params
      A = eye(3);
      
      %ekf parameters
      P_prior;
      P;
      x_hat_prior;
      x_hat;
      x_hat_I;
      R;
      Q;
      C;
      difference;
      Q_system;
      R_system
      y;
      y_hat;
      c = 1/36;  % 1/36 is Absorption coefficient of water 
      Ad =3;  %scaling constant

      R_scaling;
      Q_scaling;
      
      %controller
      u;
      
      %Previous Data
      history;
      prev_instance
      
      %coordinate_system
      local_cs;
      base_cs;
      
      %scanning parameters in a struct name scan
      scan;
      
      %coordinate matrices
      CM;

      %relative data 
      LOS;

      %motors to emulate the experimental case
      motors;
      
      %graphics to draw stuff
      graphics;
   end
   
   methods
      function obj = NodeEKF(pos,orient,scan_alternation_flag)
         if nargin > 0
            obj.position = pos;
            obj.orientation = orient/sqrt(sum(orient.^2));
            obj.orientation_angles = obj.coords2angles(obj.orientation);
            obj = obj.update_coordinate_system();
            obj = obj.initialize_motors();
            obj = obj.update_scan_parameters(scan_alternation_flag, 5, 45);  %originally 120
         end 
         
          %ekf parameters
          obj.P_prior = [10,0,0;0,900,0;0,0,900];
          %obj.Q = 100*[0.1,0,0;0,1,0;0,0,1;];
          obj.P = obj.P_prior;
          obj.x_hat_prior = [0.5,0,0]';
          obj.x_hat = obj.x_hat_prior;
          obj.x_hat_I = obj.x_hat_prior;
          %obj.R=1;
          obj.R_system = (5.15e-4)^2; %obj.Ad/500;
          obj.R_scaling = 1/obj.R_system;
          obj.Q_scaling = 100000;
          obj.R = obj.R_system;
          obj.Q_system = [0.0001,0,0;0,0.009,0;0,0,0.009;];
          obj.Q = obj.Q_system;
          obj.difference = 0;
          obj.x = [0;0;0];
          obj.y = 0;
          obj.y_hat=0;
          obj.graphics = struct();
          
          %controllers
          obj.u = [0;0];

          %Previous Data
          obj.history = DataLog();
          %obj = obj.fill_history();
          disp('Node created')
      end

      function obj = update_relative_parameters(obj, other_bot)   %parameters related to the other robot -LOS
        dir = other_bot.position - obj.position;
        obj.LOS = struct();
        obj.LOS.distance = norm(dir);
        obj.LOS.dir = dir/obj.LOS.distance;
        obj.LOS.dir_angles = obj.coords2angles(obj.LOS.dir);
        obj.LOS.local_vector = inv(obj.local_cs.Rmatrix)*obj.LOS.dir;
        obj.LOS.local_angles = obj.coords2angles(obj.LOS.local_vector);     %(x_3k, x_2k)
        
        obj.LOS.receiver_heading_vector = inv(obj.get_R_scan_global)*obj.LOS.dir;  %in the current receiver coordinates
        obj.LOS.receiver_heading_angles = obj.coords2angles(obj.LOS.receiver_heading_vector); 
        angle = acosd(obj.LOS.receiver_heading_vector(1));
        obj.LOS.receiver_direct_angle = angle;

        other_bot_receiver_heading_vector = inv(other_bot.get_R_scan_global)*(-obj.LOS.dir); %in the current receiver coordinates
        obj.LOS.other_bot_receiver_heading_angles = obj.coords2angles(other_bot_receiver_heading_vector); 
        other_angle = acosd(other_bot_receiver_heading_vector(1));
        obj.LOS.other_bot_receiver_direct_angle = other_angle;
      end

      function obj = run(obj,other_bot)
          R_scaling = obj.R_scaling; 
          Q_scaling = obj.Q_scaling; 
          obj = obj.update_relative_parameters(other_bot);
          global ind;
          global T;
          obj = obj.calculate_states();
          obj.history.x(:,:,ind) = obj.x;
          %measurement
          y = max(0,obj.get_approx_measurement() + normrnd(0,sqrt(obj.R_system)));
          obj.history.y(ind) = y;
          obj.y = y;
          obj.x_hat_prior = obj.x_hat + [0;-obj.u];
          [y_hat,C] = obj.get_output_and_jacobian(obj.x_hat_prior); 
          if obj.scan.active_phase == 1
              %Prediction phase
              obj.P_prior = 1*obj.A*obj.P*obj.A' + obj.Q*Q_scaling;

              %Analysis phase
              K = obj.P_prior*C'*inv(C*obj.P_prior*C' + obj.R*R_scaling);
              obj.x_hat = obj.x_hat_prior + K*(y-y_hat);
              obj.P = (eye(3) - K*C)*obj.P_prior;
              

              if obj.x_hat(1) < 0
                 obj.x_hat(1) = 0;
              end
              obj.x_hat_I = obj.x_hat_I + T*obj.x_hat;

              obj.difference = abs((y-y_hat)/y);
              obj.history.difference(ind) = obj.difference;

              min_ind = max(ind-2,1);
              diff_sum = sum(obj.history.difference(min_ind:ind))/3;
              pause_flag = obj.scan.pause_flag;
              if(diff_sum < 0.3)
                    %disp('here')
                    G = 0.98*pause_flag;
                    Gi = 0.1*pause_flag;
              else
                    G = 0.0;
                    Gi = 0;
              end
              u2 = -G*obj.x_hat(2)-Gi*obj.x_hat_I(2);
              u3 = -G*obj.x_hat(3)-Gi*obj.x_hat_I(3);
          else
            obj.P_prior(1,1) = obj.A(1,1)*obj.P(1,1)*obj.A(1,1) + obj.Q(1,1)*Q_scaling;
            K = obj.P_prior(1,1)*(C(1))/(C(1)*obj.P_prior(1,1)*C(1) + obj.R*R_scaling);
            obj.x_hat(1) = obj.x_hat_prior(1)+K*(y-y_hat);
            obj.x_hat_I = [0;0;0];
            obj.x_hat_I(1) =  T*obj.x_hat(1);
            obj.P(1,1) = (1 - K*C(1))*obj.P_prior(1,1);
            u2 = 0;
            u3 = 0;
          end
          
          obj.history.P_eig(ind) = max(max(abs(eig(obj.P(2:3,2:3)))));
          obj.history.x_hat(:,ind) = obj.x_hat;
          obj.history.y_hat(ind) = y_hat;
          
          %Logic for Adaptive radius
          
          obj.y_hat = y_hat;
          obj.u = -[u2; u3];
          %obj.u = [0.3;0.3];
          obj.history.net_angle(ind) = obj.LOS.receiver_direct_angle;
          obj = obj.update_self();
      end
      
      
      function obj = update_self(obj)
          obj = obj.apply_control();
          obj = obj.update_coordinate_system();
          obj = obj.update_scan_parameters();
          motor_commands = obj.generate_motor_commands();
          obj = obj.update_motors(motor_commands(1), motor_commands(2));
      end

      function obj = apply_control(obj)
            u = obj.u + normrnd([0;0],sqrt([obj.Q_system(2,2);obj.Q_system(3,3)]));
            beta = u(1);
            alpha = u(2);
            x = cosd(alpha)*cosd(beta);
            y = sind(alpha);
            z = -cosd(alpha)*sind(beta);
            obj.orientation = [obj.local_cs.x,obj.local_cs.y,obj.local_cs.z]*[x;y;z];
            obj.orientation_angles = obj.coords2angles(obj.orientation);
      end
            
      function obj = update_coordinate_system(obj)
           obj.local_cs = struct();
           obj.base_cs = struct();
           obj.local_cs.x = obj.orientation;
           obj.base_cs.y = [0;1;0];
           temp = cross(obj.local_cs.x,obj.base_cs.y);
           obj.base_cs.z = temp/norm(temp);
           obj.local_cs.z = obj.base_cs.z;
           obj.base_cs.x = cross(obj.base_cs.y,obj.base_cs.z);
           obj.local_cs.y = cross(obj.local_cs.z,obj.local_cs.x);
           obj.local_cs.Rmatrix = [obj.local_cs.x,obj.local_cs.y,obj.local_cs.z];
      end

      function obj = update_scan_parameters(obj,varargin)
          global ind;
          global num_iteration;
            if(nargin>1)    %nargin gives total number of inputs not just the length of varargin
                scan_alternation_flag = varargin{1};   %0 for rover, 1 for elevator 
                obj.scan.max_r = varargin{2};
                phi = varargin{3};
                obj.scan.phi = phi;
                counter = (360/phi)*scan_alternation_flag-1;
                obj.scan.counter = counter;
                obj.scan.active_phase = 0;
                obj.scan.pause_flag = 0;
                obj.scan.r_adaptive = 4;
                obj.scan.r = 0;
            else 
                phi = obj.scan.phi;
                num_circles = 1;
                k_factor = num_circles*360/phi;
                counter = mod(obj.scan.counter,2*k_factor) + 1;
                obj.scan.counter = counter;
                if(counter == 1)
                    obj.scan.pause_flag = 1;
                end

                if(counter == k_factor)
                    obj.scan.pause_flag = 0;
                end

                if(counter == 2*k_factor)
                    obj.scan.active_phase = 1;
                end

                if(counter == k_factor+1)
                    obj.scan.active_phase = 0;
                    %Write the scanning logic here
                    if (ind>10)
                      min_ind = max(ind-k_factor + 1,1);
                      p_ave = sum(obj.history.P_eig(min_ind:ind))/k_factor;
                      obj.scan.r_adaptive = max(2,min(obj.scan.max_r,floor((p_ave)/7000)));
                    end
                end

                obj.scan.r = obj.scan.pause_flag*obj.scan.r_adaptive;
                obj.history.r(ind) = obj.scan.r;
                
            end

            psi = (counter-1)*obj.scan.phi;
            obj.scan.psi = psi;

            if (isfield(obj.scan,'alpha'))
                obj.scan.previous_alpha = obj.scan.alpha;
            else
                obj.scan.previous_alpha = 0;
            end
            if (isfield(obj.scan,'beta'))
                obj.scan.previous_beta = obj.scan.beta;
            else
                obj.scan.previous_beta = 0;
            end
            r = obj.scan.r;
            alpha = r*sind(psi);
            obj.scan.alpha = alpha;
            beta = r*cosd(psi);
            obj.scan.beta = beta;
            obj.scan.local_vector = obj.angles2coords([alpha,beta]);
            obj.scan.vector = obj.local_cs.Rmatrix*obj.scan.local_vector;
      end

      %Motors

      function obj = initialize_motors(obj)
           obj.motors = struct();
           heading_vec = obj.orientation;
           x = heading_vec(1);
           y = heading_vec(2);
           z = heading_vec(3);

           azimuthal = atan2d(-z,x);
           elevation = atan2d(y, sqrt(x.^2 + z.^2));

           obj.motors.azimuthal = azimuthal;
           obj.motors.elevation = elevation;
           motor_angles = [obj.motors.elevation, obj.motors.azimuthal];
      end

      function h = generate_motor_commands(obj)    %u(1) = 0; u(2) = beta_control term, u(3) = alpha_control_term
        theta_motors = obj.motors.elevation;
        phi_motors = obj.motors.azimuthal;
        R_scan_global = obj.rotation_matrix([phi_motors, theta_motors]);
        obj.CM.R_scan_global= R_scan_global;

        alpha =  obj.scan.previous_alpha;
        beta = obj.scan.previous_beta;
        R_scan = obj.rotation_matrix([beta, alpha]);
        obj.CM.R_scan= R_scan;

        gamma = obj.get_roll_angle(0, theta_motors, alpha,beta);
        R_roll = obj.rotx(gamma);
        obj.CM.R_roll= R_roll;

        R_mean = R_scan_global*R_roll/R_scan;
        obj.CM.R_mean = R_mean; 
        u = obj.u;
        Ru = obj.rotation_matrix(u);
        
        next_alpha = obj.scan.alpha;
        next_beta = obj.scan.beta;
        R_scan_next = obj.rotation_matrix([next_beta, next_alpha]);
        
        R_u_global = R_mean*Ru;
        roll = atan2d(R_u_global(2,3),R_u_global(2,2));
        Ru_roll = obj.rotx(roll);
        
        next_global_angles = obj.coords2angles(R_u_global*Ru_roll*R_scan_next*[1;0;0]);
        current_global_angles = obj.coords2angles(R_scan_global*[1;0;0]);

        h = next_global_angles - current_global_angles;
      end 

      function obj = update_motors(obj,delta_elevation,delta_azimuth)
           obj.motors.azimuthal = obj.motors.azimuthal + delta_azimuth;
           obj.motors.elevation = obj.motors.elevation + delta_elevation;
           motor_angles = [obj.motors.elevation, obj.motors.azimuthal];
      end

      %Angles and transformations
      function y = get_roll_angle(obj,phi, theta, alpha,beta) 
        a = cosd(theta)*cosd(beta);
        b = cosd(theta)*sind(alpha)*sind(beta);
        c = sind(theta)*cosd(alpha)*sind(beta);
        quad_array = [a^2 + b^2,2*a*c,c^2-b^2];
        A = quad_array(1);
        B = quad_array(2);
        C = quad_array(3);
        r1  = asind(real(-(B+sqrt(B^2-4*A*C))/(2*A)));
        r2  = asind(real(-(B-sqrt(B^2-4*A*C))/(2*A)));

        r = [r1,r2];
        eq = b*cosd(r) - c - a*sind(r);
        [m,ind] = min(abs(eq));
        y = real(r(ind));
      end

      function gamma = get_net_angle(obj,alpha,beta)
          gamma = acosd(cosd(beta)*cosd(alpha));
      end

      function y = get_R_scan_global(obj)
        theta_motors = obj.motors.elevation;
        phi_motors = obj.motors.azimuthal;
        R_scan_global = obj.rotation_matrix([phi_motors, theta_motors]);
        y = R_scan_global;
      end

      function h = coords2angles(obj,X)
          x = X(1);
          y = X(2);
          z = X(3);
          
          alpha = atan2d(y, sqrt(x.^2 + z.^2));
          beta = atan2d(-z,x);
          h = [alpha, beta];    %Alpha -> Elevation, Beta -> Azimuthal
      end
      
      function h = angles2coords(obj,X)
          alpha = X(1);
          beta = X(2);
          h = [cosd(alpha)*cosd(beta); sind(alpha);-cosd(alpha)*sind(beta)];  
      end

      %Rotation Matrices

      function R = rotation_matrix(obj,X)
          phi = X(1);
          theta = X(2);
          R = obj.roty(phi)*obj.rotz(theta);
      end

      function R = rotx(obj, x)
          c = cosd(x);
          s = sind(x);
          R = [1,0,0; 0,c,-s; 0, s, c];
      end

      function R = roty(obj, x)
          c = cosd(x);
          s = sind(x);
          R = [c 0 s; 0 1 0; -s 0 c ];
      end

      function R = rotz(obj, x)
          c = cosd(x);
          s = sind(x);
          R = [c -s 0; s c 0; 0 0 1];
      end
    
      function obj = fill_history(obj)
          %This method tries to fill everything in the history except the
          %states
          global ind;
          data = obj.history;
          
          %World Pos
          data.position(:,ind) = obj.position;
          data.orientation(:,ind) = obj.orientation;
          
          %Scanning parameters
          data.bias(:,ind) = obj.bias;    %Scanning angle
          data.phi(:,ind) = obj.phi;     %Scanning angle interval
          data.scan_radius(:,ind) = obj.scan_radius; % scanning amplitude
          obj.alpha_bias = obj.scan_radius*sind(obj.bias);
          obj.beta_bias = obj.scan_radius*cosd(obj.bias); 
          
          %ekf parameters
          data.P_prior(:,:,ind) = obj.P_prior;
          data.P(:,:,ind) = obj.P;
          data.x_hat_prior(:,ind) = obj.x_hat_prior;
          data.x_hat(:,ind) = obj.x_hat;
          data.x_hat_I(:,ind) = obj.x_hat_I;
          data.x(:,:,ind) = obj.x;
          data.difference(ind) = obj.difference;
          data.y(ind) = obj.y;
          data.y_hat(ind) = obj.y_hat;
          %controller
          data.u(:,ind) = obj.u;
          
          obj.history = data;      
      end
      
      function obj = calculate_states(obj)
          d = obj.LOS.distance;
          other_ki = obj.LOS.other_bot_receiver_heading_angles;
          x1 = obj.Ad*h_comp(other_ki)*exp(-obj.c*d)/d^2;
          x2 = obj.LOS.local_angles(2);
          x3 = obj.LOS.local_angles(1);
          obj.x = [x1;x2;x3];
          
      end
      
      %Measurement functions
      function y = get_exact_measurement(obj)   %uses comp functions
         ki = obj.LOS.receiver_heading_angles;
         other_ki = obj.LOS.other_bot_receiver_heading_angles;
         d = obj.LOS.distance;
         y = obj.Ad*g_comp(ki)*h_comp(other_ki)*exp(-obj.c*d)/d^2;
      end
      
      function y = get_exact_measurement2(obj)   %uses comp functions
         ki = obj.LOS.receiver_heading_angles;
         ki = obj.get_net_angle(ki(1),ki(2));
         other_ki = obj.LOS.other_bot_receiver_heading_angles;
         other_ki = obj.get_net_angle(other_ki(1), other_ki(2));
         d = obj.LOS.distance;
         y = obj.Ad*g2(ki)*h_simple(other_ki)*exp(-obj.c*d)/d^2;
      end
       
      function y = get_approx_measurement(obj)   %uses simple functions
            ki = obj.LOS.receiver_direct_angle;
            other_ki = obj.LOS.other_bot_receiver_direct_angle;
            d = obj.LOS.distance;
            y = obj.Ad*g_simple(ki)*h_simple(other_ki)*exp(-obj.c*d)/d^2;
      end
      
      function y = get_model_measurement(obj,x)   %uses model functions
            x1 = x(1);
            x2 = x(2);
            x3 = x(3);
            ki = obj.get_net_angle(x(3),x(2));
            y = x1*g_simple(ki);
      end

      %Jacobian
      function [y,C] = get_output_and_jacobian(obj,x_hat)
            alpha = obj.scan.alpha;
            beta = obj.scan.beta;
            x1 = x_hat(1);
            x2 = x_hat(2);
            x3 = x_hat(3);
            f1 = obj.angles2coords([alpha,beta]);
            f2 = obj.angles2coords([x3,x2]);
            X = dot(f1,f2);
            grad_f2 = [-sind(x2)*cosd(x3), -cosd(x2)*sind(x3);...
                        0,                    cosd(x3); ...
                        -cosd(x2)*cosd(x3), sind(x2)*sind(x3);];
            grad_X = f1'*grad_f2;
            [g,g_d] = g_simple(acosd(X));
            if X == 1
              C = [g,0,0];     %by the limit of the overall function
            else
              C = [g,x1*g_d*(-1/sqrt(1-X^2))*grad_X];
            end       
            y = x1*g;
      end
      
      %Graphics related methods
      function obj = draw_graphics(obj)
          %obj.graphics.h_local = obj.plot_coordinate_system(obj.local_cs,'-');
          %obj.graphics.h_base = obj.plot_coordinate_system(obj.base_cs,'--');
          obj.graphics.h_scan = obj.draw_scan();
          hold on;
          obj.graphics.h_motor = obj.draw_motor();
          obj.graphics.h_estimate = obj.draw_estimate();
          
      end

      function h = plot_coordinate_system(obj,cs,style)
          origin = obj.position';
          plot3(origin(1), origin(2), origin(3),'.m', 'MarkerSize',30)
          h(1) = arrow3(origin,cs.x'*0.3,style,'r');
          hold on;
          h(2) = arrow3(origin,cs.y'*0.3,style,'g');
          h(3) = arrow3(origin,cs.z'*0.3,style,'b');
          hold on;
      end

      function h = draw_scan(obj)
          vec_scan = obj.scan.vector';
          h = arrow3(obj.position,0.25*vec_scan,'-','m');
      end
      
      function h = draw_estimate(obj)
          x = obj.x_hat;
          estimate_vector = obj.angles2coords([x(3),x(2)]); 
          vec = obj.local_cs.Rmatrix*estimate_vector;
          h = arrow3(obj.position,0.4*vec,'--','k');
      end
      
      function h = draw_motor(obj)
          alpha = obj.motors.elevation;
          beta = obj.motors.azimuthal;
          x = cosd(alpha)*cosd(beta);
          y = sind(alpha);
          z = -cosd(alpha)*sind(beta);
          vec_motor = [x,y,z];
          h = arrow3(obj.position,0.4*vec_motor,'-','k');
      end

      function h = draw_plane(obj)
        A = [-4,-0.2,4];
        B = [-4,-0.2,-4];
        C = [4,-0.2,-4];
        D = [4,-0.2,4];

        ind = 1;
        X = [A(ind),A(ind);B(ind),D(ind);C(ind),C(ind)];
        ind = 2;
        Y = [A(ind),A(ind);B(ind),D(ind);C(ind),C(ind)];
        ind = 3;
        Z = [A(ind),A(ind);B(ind),D(ind);C(ind),C(ind)];

        h_tri = fill3(X,Y,Z,'m');
        for i=1:2
            h_tri(i).EdgeColor = 'm';
            h_tri(i).EdgeAlpha = 0.5;
            h_tri(i).FaceAlpha = 0.5;
        end
        h = h_tri;
      end
      
      function obj = delete_graphics(obj)
          %delete(obj.graphics.h_local);
          %delete(obj.graphics.h_base);
          delete(obj.graphics.h_scan);
          delete(obj.graphics.h_motor);
          delete(obj.graphics.h_estimate);
          %test
      end
   end
end