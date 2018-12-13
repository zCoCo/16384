% Time Parameterized Trajectory in Jointspace
classdef JointTrajectory < handle
    properties
        jointPositions; % Matrix of Joint Angles (columns)
        jointVels;      % Matrix of Joint Velocities (columns)
        times;          % Timestamps of Each Entry in Above Matrices
    end
    
    methods
        % Compute Joint Angles and Velocities Necessary to Execute the
        % Given Workspace Trajectory
        function obj = JointTrajectory(robot, traj_w, starting_position)
            obj.times = traj_w.data.ts;
            obj.jointPositions = zeros(robot.dof, traj_w.numPts);
            obj.jointVels = zeros(robot.dof, traj_w.numPts);
            last_q = starting_position;
            for i = 1:size(traj_w.data.xs, 2)
                s = traj_w.data.xs(i);
                p = [traj_w.point(s)'; traj_w.RPY_s(s, 1,2)'];
        
                % Use IK to Convert Target Position to Joint Configuration:
                obj.jointPositions(:,i) = robot.ikd(last_q, p, robot.dof, [1,2,3,5]); % we don't care about rotation about the pointer x, yaw, idx 6.
                last_q = obj.jointPositions(:,i);
                
                % Compute Joint Velocities:
                J = robot.jacobianOf(robot.dof, obj.jointPositions(:,i));
                Lam = traj_w.RDot(s, 1,2) * traj_w.R(s, 1,2)'; % Skew Symmetric Rotation Matrix
                omega = [Lam(3,2); Lam(1,3); Lam(2,1)];
                obj.jointVels(:,i) = pinv(J)*[traj_w.velocity_s(s)'; 0;0;0]; % omega];
                fprintf('. . . %d / %d\n', i, size(traj_w.data.xs, 2));
            end
        end
        
        function p = position_t(obj, t)
            p = zeros(size(obj.jointVels,1),1);
            for i = 1 : size(obj.jointPositions,1)
                p(i) = interp1(obj.times, obj.jointPositions(i,:), t, 'spline');
            end
        end
        function v = velocity_t(obj, t)
            v = zeros(size(obj.jointVels,1),1);
            for i = 1 : size(obj.jointVels,1)
                v(i) = interp1(obj.times, obj.jointVels(i,:), t, 'spline');
            end
        end
    end
end