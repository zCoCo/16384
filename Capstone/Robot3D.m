classdef Robot3D < handle
    % Robot3D Represents a general fixed-base serial kinematic chain in 
    % SE(3) space as defined by the given Denavit-Hartenberg Parameters
    % 
    % Note: Each joint can be revolute or prismatic. Some legacy comments
    % refer to "joint angles" where it really should refer to "joint
    % states".
    
    properties (SetAccess = immutable)
        dhp, %                      - DHP Data Object
        dof, %                      - Robot's Degrees of Freedom
        link_masses, %              - Mass of Each Robot Link
        joint_masses, %             - Mass of Each Robot Joint
        end_effector_mass, %        - Mass of Robot End Effector
        joint_limits = struct(... % - Boundary Limits on Each Joint
            'mins', [], ...
            'maxs', [] ...
        )
        model %                     - RigidBodyTree Model of Robot for Visualization
        zero; %                     - Zero Vector of Length DOF
        ones; %                     - Ones Vector of Length DOF
    end
    
    methods
        % Constructor: Makes a brand new robot from the given 
        % Denavit-Hartenberg Parameter Matrix, a Vector Indicating which 
        % Column in the DHP Matrix is Actuated (0 if not) for Each Row, 
        % and Optioinal Joint Limits and Mass Parameters.
        function robot = Robot3D(dhpMat, actuatedJoints, jointmins, jointmaxs, link_masses, joint_masses, end_effector_mass)
            % Initialize Geometric Parameters:
            % Make sure that all the parameters are what we're expecting.
            
            robot.dhp = DHP(dhpMat, actuatedJoints);
                
            robot.dof = size(dhpMat, 1); % DH matrix also includes tool frame
            % For later convenience:
            robot.zero = zeros(robot.dof, 1);
            robot.ones = ones(robot.dof, 1);
            
            % Initialize Joint Limits:
            if nargin > 2
                if length(jointmins) ~= robot.dof || length(jointmaxs) ~= robot.dof
                    error('Robot Must Have Same Number of Joint Limits as Dimensions');
                end
                robot.joint_limits.mins = jointmins;
                robot.joint_limits.maxs = jointmaxs;
            else % Set up default functionally unbounded joint limits:
                robot.joint_limits.mins = zeros(robot.dof,1);
                robot.joint_limits.maxs = zeros(robot.dof,1);
                for i = 1:robot.dof
                    if robot.dhp.actuatedJoints(i) == 3 % is prismatic?
                        robot.joint_limits.mins(i) = -Inf;
                        robot.joint_limits.maxs(i) = Inf;
                    else
                        robot.joint_limits.mins(i) = -2*pi;
                        robot.joint_limits.maxs(i) = 2*pi;
                    end
                end
            end
            
            % Initialize Mass Parameters:
            if nargin > 4
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
            
            % Create Visualization Model:
            % Create a rigid body tree object to build the robot.
            robot.model = robotics.RigidBodyTree;
            
            % Create Link 1 and Attach to Base:
            body = robotics.RigidBody('body1');
            if robot.dhp.actuatedJoints(1) == 3
                joint = robotics.Joint('joint1', 'prismatic');
            elseif robot.dhp.actuatedJoints(1) == 0
                joint = robotics.Joint('joint1', 'fixed');
            else
                joint = robotics.Joint('joint1', 'revolute');
            end

            joint.setFixedTransform(robot.dhp.data.mat(1,:),'dh');

            body.Joint = joint;
            robot.model.addBody(body, 'base');

            % Create and Attach Subsequent Links:
            for i = 2:robot.dof
                body = robotics.RigidBody( strcat('body', num2str(i)) );
                if robot.dhp.actuatedJoints(i) == 3
                    joint = robotics.Joint( strcat('joint', num2str(i)), 'prismatic' );
                elseif robot.dhp.actuatedJoints(i) == 0
                    joint = robotics.Joint( strcat('joint', num2str(i)), 'fixed' );
                else
                    joint = robotics.Joint( strcat('joint', num2str(i)), 'revolute' );
                end

                joint.setFixedTransform(robot.dhp.data.mat(i,:),'dh');

                body.Joint = joint;
                robot.model.addBody( body, strcat('body', num2str(i-1)) );
            end
            
            showdetails(robot.model);
        end % ctor
       
        % Returns the forward kinematic map for each frame, one for each 
        % link. Link i is given by frames(:,:,i), 
        % and the end effector frame is frames(:,:,end).
        function frames = forward_kinematics(robot, thetas)
            frames = zeros(4,4,robot.dof); % Set of homogeneous transforms to be populated
            
            % Get the DHP Data Package at the Given Joint Configuration:
            dhpData = robot.dhp.atConfig(thetas);
            
            % Set frame for first joint to the base transform:
            frames(:,:,1) = DHP.transform(dhpData, 1);
            
            i = 2;
            while(i <= robot.dof) % For each joint frame, i, and the tool frame
                % Compute total DH transform H_(i-1)^0 * (H_i^(i-1)):
                frames(:,:,i) = frames(:,:,i-1) * DHP.transform(dhpData, i);
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
            % Get the DHP Data Package at the Given Joint Configuration:
            dhpData = robot.dhp.atConfig(thetas);
            
            % Set frame for first joint to the base transform:
            frame = DHP.transform(dhpData, 1);
            
            i = 2;
            while(i <= f) % For each joint frame, i, and the tool frame
                % Compute local DH transform H_(i-1)^0 * (H_i^(i-1)):
                frame = frame * DHP.transform(dhpData, i);
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
            
            if(R(1,1) == 0 || R(2,1) == 0)
                yaw = atan2(R(1,2), R(2,2));
                pitch = pi/2;
                roll = 0;
            else
                yaw = atan2(H_0_i(3,2), H_0_i(3,3));
                pitch = atan2(-H_0_i(3,1), sqrt(H_0_i(3,2)^2 + H_0_i(3,3)^2));
                roll = atan2(H_0_i(2,1), H_0_i(1,1));
            end
           
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
            % Returns the SE(3) Geometric Jacobian for each frame.

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
        
        function J = jacobianOf(robot, f, thetas)
            % Returns the SE(3) Geometric Jacobian for the frame with
            % index f (faster than #jacobians if only one J is needed).
            % Because this calls fk on each call, it is still faster to 
            % call jacobians if you need more than one or two jacobians for
            % a computation.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
               error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end

            fk = robot.fk(thetas);
            J = zeros(6,robot.dof);
            
            o_j = fk(1:3,4,f); % Position of target frame origin w.r.t. {0} = H*[0;0;0;1] 

            % Create first column of jacobian for joint frame j:
            z = [0;0;1]; % Orientation of z-axis for {0}
            o = [0;0;0]; % Position of origin for {0}
            J(1:3, 1) = cross(z, (o_j - o));
            J(4:6, 1) = z;
                
            i = 2;
            while(i <= robot.dof) % For each column of the jacobian, i
                H = fk(:,:,i-1); % Homogenous transform to joint i-1
                z = H(1:3,3); % Orientation of previous z-axis w.r.t. {0} = H*[0;0;1;0]
                o = H(1:3,4); % Position of previous origin w.r.t. {0} = H*[0;0;0;1]

                J(1:3, i) = cross(z, (o_j - o));
                J(4:6, i) = z;
            i = i + 1;
            end
        end % #jacobianOf
        
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
        function thetas = inverse_kinematics_decoupled(robot, initial_thetas, goal_position, joint, idxs)
            % Make sure that all the parameters are what we're expecting.
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
    
        % Visualizes the Robot as a RigidBodyTree in the Given Joint-Angle 
        % Configuration, q.
        function f = visualize(robot, q)
            persistent fig boundary;
            if isempty(fig) || ~ishandle(fig)
                % Only setup figure once:
                fig = figure();
                
                % Set Really Rough Bounding Box:
                if isempty(boundary)
                    maxExtent = robot.ee(zeros(robot.dof, 1));
                    boundary = norm(maxExtent(1:3));
                end
                if boundary == 0
                    % This shouldn't happen except in weird cases.
                    boundary = sum(robot.dhp.data.as) + sum(robot.dhp.data.ds);
                end
                axis([-boundary,boundary,-boundary,boundary,-boundary,boundary])
                axis off
            end
            figure(fig);
            
            if length(q) ~= robot.dof
                error('Invalid Joint Configuration State. Vector should have as many cells as Robot DOF, which is %d.', robot.dof);
            end
            
            % Create Config Structure Array:
            cfg = robot.model.homeConfiguration();
            for i = 1:robot.dof
                cfg(i).JointPosition = q(i) + robot.dhp.data.mat(i, robot.dhp.actuatedJoints(i));
            end
            
            % Plot Robot Body:
            robot.model.show(cfg);
            
            f = fig;
        end % #visualize
        
        % ** DEPRECATED ** Use #visualize instead.
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