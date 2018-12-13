classdef JointTrajectory < handle
    properties
        jointPositions; % Matrix of Joint Angles (columns)
        jointVel;       % Matrix of Joint Velocities (columns)
    end
    
    methods
        % Compute Joint Angles and Velocities Necessary to Execute the
        % Given Workspace Trajectory
        function obj = JointTrajectory(robot, traj_w, starting_position)
            jointPositions = zeros(robot.dof, traj_w.numPts);
            last_q = starting_position;
            for i = 1:traj_w.numPts
                s = dist(i);
                p = [traj.points(i,:)'; traj.RPY_s(s, 1,2)'];
        
                % Use IK to Convert Target Position to Joint Configuration:
                jointPositions(:,i) = robot.ikd(last_q, p, robot.dof, [1,2,3,5]); % we don't care about rotation about the pointer x, yaw, idx 6.
                last_q = jointPositions(:,i);
                
                % Compute Joint Velocities:
                J = rc.robot.jacobianOf(rc.robot.dof, rc.command.q);
                rc.command.v = pinv(J)*V;
                J = 
                
            end
            for s = linspace(0, traj.dist(end), 100)
                % Get Target Workspace Position:

                % Update Data Logging:
                diffSum = diffSum + norm(p-X);
                numUpdates = numUpdates + 1

                robot.visualize(q);
                hold on
                    scatter3(wps_w(:,1), wps_w(:,2), wps_w(:,3));
                    scatter3(log_w(:,1), log_w(:,2), log_w(:,3));
                hold off
                pause(0.03); % CPU Relief
            end
        end
    end
end