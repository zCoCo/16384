classdef JointTrajectory < handle
    properties
        jointPositions; % Matrix of Joint Angles (columns)
        jointVels;      % Matrix of Joint Velocities (columns)
    end
    
    methods
        % Compute Joint Angles and Velocities Necessary to Execute the
        % Given Workspace Trajectory
        function obj = JointTrajectory(robot, traj_w, starting_position)
            obj.jointPositions = zeros(robot.dof, traj_w.numPts);
            obj.jointVels = zeros(robot.dof, traj_w.numPts);
            last_q = starting_position;
            for i = 1:traj_w.numPts
                s = dist(i);
                p = [traj.points(i,:)'; traj.RPY_s(s, 1,2)'];
        
                % Use IK to Convert Target Position to Joint Configuration:
                obj.jointPositions(:,i) = robot.ikd(last_q, p, robot.dof, [1,2,3,5]); % we don't care about rotation about the pointer x, yaw, idx 6.
                last_q = obj.jointPositions(:,i);
                
                % Compute Joint Velocities:
                J = robot.jacobianOf(robot.dof, obj.jointPositions(:,i));
                Lam = traj.RDot * traj.R(s, 1,2)'; % Skew Symmetric Rotation Matrix
                omega = [Lam(3,2); Lam(1,3); Lam(2,1)];
                obj.jointVels(:,i) = pinv(J)*[traj.velocity(s)'; omega];
            end
        end
    end
end