classdef Robot3D < handle
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dhp = struct(... % Denavit-Hartenberg Parameters for each Joint
            'as', [], ...
            'alphas', [], ...
            'ds', [], ...
            'mat', []... % matrix of params
        );
        dof
        link_lengths
        link_masses
        joint_masses
        end_effector_mass
        joint_limits = struct(...
            'mins', [], ...
            'maxs', [] ...
        );
    end
    
    methods
        % Constructor: Makes a brand new robot with geometric and optional 
        % mass parameters.
        function robot = Robot3D(dhp, jointmins, jointmaxs, link_masses, joint_masses, end_effector_mass)
            % Initialize Geometric Parameters:
            % Make sure that all the parameters are what we're expecting.
            if size(dhp, 2) ~= 4
                error('Invalid Denavit-Hartenberg Parameter Matrix: should have four columns');
            end
                
            robot.dof = size(dhp, 1); % DH matrix also includes tool frame
            
            if length(jointmins) ~= robot.dof || length(jointmaxs) ~= robot.dof
                error('Robot Must Have Same Number of Joint Limits as Dimensions');
            end
            robot.joint_limits.mins = jointmins;
            robot.joint_limits.maxs = jointmaxs;
            
            robot.dhp.mat = dhp;
                robot.dhp.as = dhp(:,1);
                robot.dhp.alphas = dhp(:,2);
                robot.dhp.ds = dhp(:,3);
                
            % Initialize Mass Parameters:
            if nargin > 1
                if size(link_masses, 2) ~= 1
                   error('Invalid link_masses: Should be a column vector, is %dx%d.', size(link_masses, 1), size(link_masses, 2));
                end

                if size(joint_masses, 2) ~= 1
                   error('Invalid joint_masses: Should be a column vector.');
                end

                if ~isnumeric(end_effector_mass)
                   error('Invalid end_effector_mass: Should be a number.'); 
                end

                if size(link_masses, 1) ~= robot.dof
                    error('Invalid number of link masses: should match number of link lengths.');
                end

                if size(joint_masses, 1) ~= robot.dof
                    error('Invalid number of joint masses: should match number of link lengths. Did you forget the base joint?');
                end

                robot.link_masses = link_masses;
                robot.joint_masses = joint_masses;
                robot.end_effector_mass = end_effector_mass;
            else
                robot.link_masses = zeros(robot.dof);
                robot.joint_masses = zeros(robot.dof);
                robot.end_effector_mass = 0;
            end % nargin>1
        end % ctor
       
        % Returns the forward kinematic map for each frame, one for each 
        % link. Link i is given by frames(:,:,i), 
        % and the end effector frame is frames(:,:,end).
        function frames = forward_kinematics(robot, thetas)
            if size(thetas, 2) ~= 1
                error('Expecting a column vector of joint angles.');
            end
            
            if size(thetas, 1) ~= robot.dof
                error('Invalid number of joints: %d found, expecting %d', size(thetas, 1), robot.dof);
            end
            
            frames = zeros(4,4,robot.dof); % Set of homogeneous transforms to be populated

            % Shorthand to Ease Reading:
            ct = @(nn) cos(thetas(nn)); % Returns the cos of joint angle nn
            st = @(nn) sin(thetas(nn)); % Returns the sin of joint angle nn
            ca = @(nn) cos(robot.dhp.alphas(nn)); % Returns the cos of the DH alpha for frame nn
            sa = @(nn) sin(robot.dhp.alphas(nn)); % Returns the sin of the DH alpha for frame nn
            
            a = @(nn) robot.dhp.as(nn); % Returns the DH a offset for frame nn
            d = @(nn) robot.dhp.ds(nn); % Returns the DH d offset for frame nn
            
            % Compute frame for first joint (1st frame):
            frames(:,:,1) = [...
                ct(1), -st(1)*ca(1), st(1)*sa(1), a(1)*ct(1);...
                st(1), ct(1)*ca(1), -ct(1)*sa(1), a(1)*st(1);...
                0, sa(1), ca(1), d(1);...
                0,0,0,1 ...
            ];
            i = 2;
            while(i <= robot.dof) % For each joint frame, i, and the tool frame
                % Compute local DH transform (H_i^(i-1))
                A = [...
                    ct(i), -st(i)*ca(i), st(i)*sa(i), a(i)*ct(i);...
                    st(i), ct(i)*ca(i), -ct(i)*sa(i), a(i)*st(i);...
                    0, sa(i), ca(i), d(i);...
                    0,0,0,1 ...
                ];
                frames(:,:,i) = frames(:,:,i-1) * A;
            i = i+1;
            end
        end % #forward_kinematics
       
        % Shorthand for returning the forward kinematics.
        function fk = fk(robot, thetas)
            fk = robot.forward_kinematics(thetas);
        end
        
        % Returns the forward kinematic map for only the given frame, f.
        % More efficient if only a specific frame is needed.
        function frame = forward_kinematics_frame(robot, f, thetas)
            if size(thetas, 2) ~= 1
                error('Expecting a column vector of joint angles.');
            end
            
            if size(thetas, 1) ~= robot.dof
                error('Invalid number of joints: %d found, expecting %d', size(thetas, 1), robot.dof);
            end

            % Shorthand to Ease Reading:
            ct = @(nn) cos(thetas(nn)); % Returns the cos of joint angle nn
            st = @(nn) sin(thetas(nn)); % Returns the sin of joint angle nn
            ca = @(nn) cos(robot.dhp.alphas(nn)); % Returns the cos of the DH alpha for frame nn
            sa = @(nn) sin(robot.dhp.alphas(nn)); % Returns the sin of the DH alpha for frame nn
            
            a = @(nn) robot.dhp.as(nn); % Returns the DH a offset for frame nn
            d = @(nn) robot.dhp.ds(nn); % Returns the DH d offset for frame nn
            
            % Compute frame for first joint (1st frame):
            frame = [...
                ct(1), -st(1)*ca(1), st(1)*sa(1), a(1)*ct(1);...
                st(1), ct(1)*ca(1), -ct(1)*sa(1), a(1)*st(1);...
                0, sa(1), ca(1), d(1);...
                0,0,0,1 ...
            ];
            i = 2;
            while(i <= f) % For each joint frame, i, and the tool frame
                % Compute local DH transform (H_i^(i-1))
                A = [...
                    ct(i), -st(i)*ca(i), st(i)*sa(i), a(i)*ct(i);...
                    st(i), ct(i)*ca(i), -ct(i)*sa(i), a(i)*st(i);...
                    0, sa(i), ca(i), d(i);...
                    0,0,0,1 ...
                ];
                frame = frame * A;
            i = i+1;
            end
        end % #forward_kinematics_frame
       
        % Shorthand for returning the forward kinematic map to a specific
        % frame, f.
        function fkf = fkf(robot, f, thetas)
            fkf = robot.forward_kinematics_frame(f, thetas);
        end
        
        % Returns position for the frame with index idx given a set of 
        % joint angles.
        function p = position(robot, idx, thetas)
            % Find the transform to the given frame.
            H_0_i = robot.fkf(idx, thetas);
           
            % Extract the components of the position and orientation.
            x = H_0_i(1,4);
            y = H_0_i(2,4);
            z = H_0_i(3,4);
            yaw = atan2(H_0_i(3,2), H_0_i(3,3));
            pitch = atan2(-H_0_i(3,1), sqrt(H_0_i(3,2)^2 + H_0_i(3,3)^2));
            roll = atan2(H_0_i(2,1), H_0_i(1,1));
           
            % Pack them up nicely.
            p = [x; y; z; roll; pitch; yaw];
        end % #position
        % Shorthand for returning the position of the frame with index idx 
        % given a set of joint angles.
        function p = p(robot, idx, thetas)
            p = robot.position(idx, thetas);
        end
        
        
        % Returns position for the end effector given a set of joint
        % angles.
        function ee = end_effector(robot, thetas)
            ee = robot.position(robot.dof, thetas);
        end % #end_effector
        % Shorthand for returning the end effector position and orientation. 
        function ee = ee(robot, thetas)
            ee = robot.end_effector(thetas);
        end
        
        
        function jacobians = jacobians(robot, thetas)
            % Returns the SE(3) Analytical Jacobian for each frame.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
               error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end

            fk = robot.fk(thetas);
            jacobians = zeros(6,robot.dof,robot.dof);
            
            j = 1;
            while(j <= robot.dof) % For each joint frame j and the tool frame
                o_j = fk(1:3,4,j); % Position of target frame origin w.r.t. {0} = H*[0;0;0;1] 
                
                % Create first column of jacobian for joint frame j:
                z = [0;0;1]; % Orientation of z-axis for {0}
                o = [0;0;0]; % Position of origin for {0}
                jacobians(1:3, 1, j) = cross(z, (o_j - o));
                jacobians(4:6, 1, j) = z;
                
                i = 2;
                while(i <= robot.dof) % For each column of the jacobian, i
                    H = fk(:,:,i-1); % Homogenous transform to joint i-1
                    z = H(1:3,3); % Orientation of previous z-axis w.r.t. {0} = H*[0;0;1;0]
                    o = H(1:3,4); % Position of previous origin w.r.t. {0} = H*[0;0;0;1]

                    jacobians(1:3, i, j) = cross(z, (o_j - o));
                    jacobians(4:6, i, j) = z;
                i = i + 1;
                end
            j = j + 1;
            end
        end % #jacobians
        
        function thetas = inverse_kinematics(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end

            if (size(goal_position, 1) ~= 6 || size(goal_position, 2) ~= 1)
                error('Invalid goal_position: Should be a 6 length column vector, is %dx%d.', size(goal_position, 1), size(goal_position, 2));
            end
            
            function c = cost(q)
                pe = robot.ee(q);
                c = (pe-goal_position)' * (pe-goal_position);
            end
            
            thetas = fmincon(cost, initial_thetas, [],[],[],[], robot.joint_limits.mins, robot.joint_limits.maxs);
        end % #inverse_kinematics
        % Shorthand for preforming the inverse kinematics.
        function ik = ik(robot, initial_thetas, goal_position)
            ik = inverse_kinematics(robot, initial_thetas, goal_position);
        end
        
        % Performs Inverse Kinematics for System that was Kinematically
        % Decoupled using External Data. This entails only solving IK for
        % the joint center and only caring about the given dimension
        % indices, idxs. Additionally, this function can also perform IK
        % for complete non-decoupled systems where one or several dimension 
        % indices (ex. roll=4) doesn't matter.
        function thetas = inverse_kinematics_decoupled(robot, initial_thetas, goal_position, joint, idxs)% Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end

            if (size(goal_position, 1) ~= 6 || size(goal_position, 2) ~= 1)
                error('Invalid goal_position: Should be a 6 length column vector, is %dx%d.', size(goal_position, 1), size(goal_position, 2));
            end
            
            function c = cost(q)
                pe = robot.p(joint, q);
                D = pe(idxs)-goal_position(idxs);
                c = D' * D;
            end
            
            thetas = fmincon(cost, initial_thetas, [],[],[],[], robot.joint_limits.mins, robot.joint_limits.maxs);
        end % #inverse_kinematics_decoupled
        
        % Shorthand for performing the decoupled inverse kinematics.
        function ikd = ikd(robot, initial_thetas, goal_position, joint, idxs)
            ikd = inverse_kinematics_decoupled(robot, initial_thetas, goal_position, joint, idxs);
        end
    
        % ** DEPRECATED **
        % Plot the robots frames for the given configuration on the given
        % figure (if given)
        function plot(robot, thetas, fig)
            % Set up plot:
            persistent color_x color_y color_z colors color_body;
            if isempty(color_x)
              color_x = 'r'; color_y = 'b'; color_z = 'k';
              colors = [color_x, color_y, color_z];
              color_body = 'g';
            end
            
            if nargin > 2
                figure(fig);
            else
                figure();
            end
            
            fk = robot.fk(thetas); % Get Forward Kinematics:
            
            % Plot zero frame:
            o = [0;0;0];
            plot3([0;1],[0;0],[0;0], color_x);
            plot3([0;0],[0;1],[0;0], color_y);
            plot3([0;0],[0;0],[0;1], color_z);
            hold on
            
            % Plot other frames
            for frame = 1:robot.dof
                o_prev = o;
                o = fk(1:3,4, frame);
                plot3([o(1); o_prev(1)], [o(2); o_prev(2)], [o(3); o_prev(3)], color_body);
                
                axes = fk(1:3,1:3, frame);
                for i = 1:3
                    e = o + axes(:,i);
                    plot3([o(1); e(1)], [o(2); e(2)], [o(3); e(3)], colors(i));
                end
            end
            hold off
        end % #plot

    end % methods
end % classdef Robot3D