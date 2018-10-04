classdef Robot
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dof
        link_lengths
        link_masses
        joint_masses
        end_effector_mass
    end
    
    methods
        % Constructor: Makes a brand new robot with the specified parameters.
        function robot = Robot(link_lengths, link_masses, joint_masses, end_effector_mass)
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(link_lengths, 2) ~= 1
               error('Invalid link_lengths: Should be a column vector, is %dx%d.', size(link_lengths, 1), size(link_lengths, 2));
            end
            
            if size(link_masses, 2) ~= 1
               error('Invalid link_masses: Should be a column vector, is %dx%d.', size(link_masses, 1), size(link_masses, 2));
            end
            
            if size(joint_masses, 2) ~= 1
               error('Invalid joint_masses: Should be a column vector.');
            end
            
            if ~isnumeric(end_effector_mass)
               error('Invalid end_effector_mass: Should be a number.'); 
            end
            
            robot.dof = size(link_lengths, 1);
            
            if size(link_masses, 1) ~= robot.dof
                error('Invalid number of link masses: should match number of link lengths.');
            end
            
            if size(joint_masses, 1) ~= robot.dof
                error('Invalid number of joint masses: should match number of link lengths. Did you forget the base joint?');
            end
            
            robot.link_lengths = link_lengths;
            robot.link_masses = link_masses;
            robot.joint_masses = joint_masses;
            robot.end_effector_mass = end_effector_mass;  
        end
       
        % Returns the forward kinematic map for each frame, one for the base of
        % each link, and one for the end effector. Link i is given by
        % frames(:,:,i), and the end effector frame is frames(:,:,end).
        function frames = forward_kinematics(robot, thetas)
            if size(thetas, 2) ~= 1
                error('Expecting a column vector of joint angles.');
            end
            
            if size(thetas, 1) ~= robot.dof
                error('Invalid number of joints: %d found, expecting %d', size(thetas, 1), robot.dof);
            end
            
            % Allocate a variable containing the transforms from each frame
            % to the base frame.
            n = robot.dof;
            frames = zeros(3,3,n+1); % Set of homogeneous transforms to be populated
            % The transform from the base of link 'i' to the base frame (H^0_i)
            % is given by the 3x3 matrix frames(:,:,i).

            % The transform from the end effector to the base frame (H^0_i) is
            % given by the 3x3 matrix frames(:,:,end).
            
            % Shorthand to Ease Reading:
            c = @(xx) cos(thetas(xx)); % Returns the cos of joint angle n
            s = @(xx) sin(thetas(xx)); % Returns the sin of  joint angle n
            l = @(xx) robot.link_lengths(xx); % Returns the length of the nth link

            frames(:,:,1) = [c(1), -s(1), 0; s(1), c(1), 0; 0,0,1];
            i = 2;
            while(i<=n) % while loops are faster than for in MATLAB
                frames(:,:,i) = frames(:,:,i-1) * [c(i), -s(i), l(i-1); s(i), c(i), 0; 0,0,1]; % translate then rotate
            i = i+1;
            end
            frames(:,:,n+1) = frames(:,:,n) * [1, 0, l(n); 0,1,0; 0,0,1]; % translate to end effector
        end
       
        % Shorthand for returning the forward kinematics.
        function fk = fk(robot, thetas)
            fk = robot.forward_kinematics(thetas);
        end
       
        % Returns [x; y; theta] for the end effector given a set of joint
        % angles. 
        function ee = end_effector(robot, thetas)
            % Find the transform to the end-effector frame.
            frames = robot.fk(thetas);
            H_0_ee = frames(:,:,end);
           
            % Extract the components of the end_effector position and
            % orientation.
            x = H_0_ee(1,3);
            y = H_0_ee(2,3);
            th = atan2(H_0_ee(2, 1), H_0_ee(1, 1));
           
            % Pack them up nicely.
            ee = [x; y; th];
        end
       
        % Shorthand for returning the end effector position and orientation. 
        function ee = ee(robot, thetas)
            ee = robot.end_effector(thetas);
        end
        
        function jacobians = jacobians(robot, thetas)
            % Returns the SE(2) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
               error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end

            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            jacobians = zeros(3,robot.dof,robot.dof+1); 
            epsilon = 0.001;
           
            % The jacobian for frame 'i' (as defined by the forward kinematics
            % function above) is the 3xn matrix jacobians(:,:,i), where n is the
            % number of degrees of freedom.

            % In this function, compute the _numerical derivative_ for each
            % element of the Jacobian.

            % Hint: The third row of the Jacobian (the resulting frame angular
            % velocity given the joint angular velocity) is either 1 or 0,
            % depending on whether or not that joint contributes to that frame's
            % rotation.

            % Hint: jacobians(1,j,frame) is the partial differential of
            % the 'x' coordinate of 'frame' (with respect to the base frame) due
            % to motion of the joint j.  Mathematically, this is
            % d{x^0_frame}/d{theta_j}.
            %
            % To find this quantity numerically, one can compute the 'numerical
            % derivative' instead of the 'analytic derivative'.  This is
            % essentially approximating g'(x), the derivative of some function
            % g(x) at x, by computing (g(x+dx) - g(x-dx)) / (2*dx).
            %
            % Applying this to the task of finding jacobians(1,j,frame), we know
            % that:
            %   x represents theta
            %   g is the forward kinematics to this point, or the (1, 3, frame)
            %     index of the result of calling fk.
            %   dx is a vector of zeros for each joint except for joint j, and a
            %     small value for j (use the 'epsilon' defined above).
            %
            % This same logic can be used to find the y component, or
            % jacobians(2,j,frame).

            % Reminder: to get the x and y coordinates of the origin of each
            % frame, one could call the following commands (essentially pulling
            % out the translation term from the homogeneous transform to each
            % frame)
            %
            % base_frames = robot.fk(thetas); % Find the frames
            % frames_x = base_frames(1,3,:); % Get x values of the origins
            % frames_y = base_frames(2,3,:); % Get y values of the origins

% --------------- BEGIN STUDENT SECTION ----------------------------------
            for joint = 1 : robot.dof
   
                dth = zeros(robot.dof,1); dth(joint) = epsilon;
                dFdth = (robot.fk(thetas+dth) - robot.fk(thetas-dth)) ./ (2*epsilon);
  
                for frame = 1 : size(dFdth,3)
                    jacobians(1, joint, frame) = dFdth(1,3,frame);
                    jacobians(2, joint, frame) = dFdth(2,3,frame);
                    jacobians(3, joint, frame) = dFdth(3,3,frame);
                end
            end
% --------------- END STUDENT SECTION ------------------------------------
        end % #jacobians
        
        function thetas = inverse_kinematics(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end

            if (size(goal_position, 1) ~= 2 && size(goal_position, 1) ~= 3) || ...
               size(goal_position, 2) ~= 1
                error('Invalid goal_position: Should be a 2 or 3 length column vector, is %dx%d.', size(goal_position, 1), size(goal_position, 2));
            end

            % Allocate a variable for the joint angles during the optimization;
            % begin with the initial condition
            thetas = initial_thetas;

            % Step size for gradient update
            step_size = 0.1;

            % Once the norm (magnitude) of the computed gradient is smaller than
            % this value, we stop the optimization
            stopping_condition = 0.00005;

            % Also, limit to a maximum number of iterations.
            max_iter = 100;
            num_iter = 0;

% --------------- BEGIN STUDENT SECTION ----------------------------------
            % Run gradient descent optimization
            while (num_iter < max_iter)

                % Compute the gradient for either an [x;y] goal or an
                % [x;y;theta] goal, using the current value of 'thetas'.
                % TODO fill in the gradient of the squared distance cost function
                % HINT use the answer for theory question 2, the
                % 'robot.end_effector' function, and the 'robot.jacobians'
                % function to help solve this problem
                if (size(goal_position, 1) == 2) % [x;y] goal
                    cost_gradient = zeros(robot.dof, 1);
                else % [x;y;theta] goal
                    cost_gradient = zeros(robot.dof, 1);
                end

                % Update 'thetas'
                % TODO

                % Check stopping condition, and return if it is met.
                % TODO

                num_iter = num_iter + 1;
            end
% --------------- END STUDENT SECTION ------------------------------------
        end % #inverse_kinematics
        
        % Shorthand for preforming the inverse kinematics.
        function ik = ik(robot, initial_thetas, goal_position)
            ik = robot.inverse_kinematics(robot, initial_thetas, goal_position);
        end
    end % methods
end % classdef