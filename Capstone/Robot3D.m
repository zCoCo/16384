classdef Robot3D
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
            % Returns the SE(3) Jacobian for each frame.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
               error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end

            fk = robot.fk(thetas);
            o_n = fk(1:3,4,end); % Position of end-effectorr origin w.r.t. {0} = H*[0,0,0,1]^T 
            
            j = 1;
            while(j <= robot.dof) % For each joint j
                i = 1;
                while(i <= robot.dof) % For each column of the jacobian, i
                    H = fk(:,:,j); % Homogenous transform to joint j-1
                    z = H(1:3,3); % Orientation of previous z-axis w.r.t. {0} = H*[0,0,1,0]^T 
                    o = H(1:3,4); % Position of previous origin w.r.t. {0} = H*[0,0,0,1]^T

                    jacobians(1:3, j, 
                    dth = zeros(robot.dof,1); dth(joint) = epsilon;
                    dFdth = (robot.fk(thetas+dth) - robot.fk(thetas-dth)) ./ (2*epsilon);

                    for frame = 1 : size(dFdth,3)
                        jacobians(1, joint, frame) = dFdth(1,3,frame);
                        jacobians(2, joint, frame) = dFdth(2,3,frame);
                        jacobians(3, joint, frame) = dFdth(3,3,frame);
                    end
                i = i + 1;
                end
            j = j + 1;
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
            max_iter = 900;
            num_iter = 0;

% --------------- BEGIN STUDENT SECTION ----------------------------------
            % Run gradient descent optimization
            while (num_iter < max_iter)
                Js = robot.jacobians(thetas);
                Jinv = Js(:,:,end)';
                pe = robot.ee(thetas);
                % Compute the gradient for either an [x;y] goal or an
                % [x;y;theta] goal, using the current value of 'thetas'.
                % TODO fill in the gradient of the squared distance cost function
                % HINT use the answer for theory question 2, the
                % 'robot.end_effector' function, and the 'robot.jacobians'
                % function to help solve this problem
                if (size(goal_position, 1) == 2) % [x;y] goal
                    cost_gradient = 2 * Jinv(:, 1:2) * (pe(1:2) - goal_position);
                else % [x;y;theta] goal
                    cost_gradient = 2 * Jinv(:, 1:3) * (pe(1:3) - goal_position);
                end

                thetas = thetas - cost_gradient * step_size;

                if( norm(cost_gradient) < stopping_condition )
                    break;
                end
                
                num_iter = num_iter + 1;
            end
% --------------- END STUDENT SECTION ------------------------------------
        end % #inverse_kinematics
        
        % Shorthand for preforming the inverse kinematics.
        function ik = ik(robot, initial_thetas, goal_position)
            ik = inverse_kinematics(robot, initial_thetas, goal_position);
        end
    end % methods
end % classdef 3D