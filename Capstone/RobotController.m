% Controls a Robot Described by a Given Robot3D Object. This involves
% either managing connection to and control of real hardware (Hebi
% actuators) or running a VisualizeRobotics (VR) simulation.
classdef RobotController < handle
    properties(SetAccess = immutable)
        robot; %                - Robot3D object being controlled
        robotHardware; %        - Close to the Metal Hebi Controller for Issuing Commands to the Robot
    end % immutable properties
    
    properties(SetAccess = private, GetAccess = public)
        commState = struct(... %    - Last State Commanded to Robot
            'q', [], ... %              - Joint State Vector
            'v', [], ... %              - Joint Velocity Vector
            't', [] ... %               - Joint Torque Vector
        );
        currState = struct(... %    - Current Robot State from Feedback
            'q', [], ... %              - Joint State Vector
            'v', [], ... %              - Joint Velocity Vector
            't', [] ... %               - Joint Torque Vector
        );
        sim = true; %               - Whether this is controlling an actual robot or a VR sim.
        clock; %                    - Clock Used for Timing Events (ie. Trajectories)
    end % private,public properties
    
    properties(SetAccess = public, GetAccess = public)
        command = struct(... %  - Command to be Sent to the Robot
            'q', [], ... %          - Joint State Vector
            'v', [], ... %          - Joint Velocity Vector
            't', [], ... %          - Joint Torque Vector
            'sent', true ... %      - Whether This Command has been Sent Yet
        );
    end % public,public properties
    
    methods
        % Instantiates a RobotConroller for the Given Robot3D Object
        function rc = RobotController(rob)
            rc.robot = rob;
            rc.clock = Clock(); % Set Default Clock
        end % ctor
        
        % Initializes Connection to Robot Hardware
        function initializeHardwareConnection(rc)
            HebiLookup.setLookupAddresses('*');
            HebiLookup.clearModuleList();
            HebiLookup.clearGroups();
            pause(3);
            
            rc.robotHardware = HebiLookup.newGroupFromNames('Robot B',{'J1','J2', 'J3', 'J4', 'J5'});
            rc.robotHardware.setCommandLifetime(2);
            
            gains_struct_wrapper = load('gains_file.mat'); %contains gains_struct
            gains_struct = gains_struct_wrapper.gains_struct;

            gains_struct.controlStrategy = 4 * rc.robot.ones;
            gains_struct.positionKp = [0.75 3 2 2 2];
            gains_struct.positionKi = [0.005 0 0 0 0];
            gains_struct.positionKd = [0.005 0 0.03 0 0];
            % display(gains_struct) will show you other fields you can modify
            rc.robotHardware.set('gains', gains_struct); % note this is the set function, not send
        end % #initializeHardwareConnection
        
        % Updates the Robot's State from Feedback
        function updateState(rc)
            if rc.sim
                if isempty(rc.currState.q)
                    home = rc.robot.model.homeConfiguration;
                    rc.currState = [home(:).JointPosition];
                else
                    rc.currState.q = rc.commState.q;
                    rc.currState.v = rc.commState.v;
                    rc.currState.t = rc.commState.t;
                end    
            else
                warning('#updateState Not Implemented for Robot Hardware');
                % @TODO
            end
        end % #updateState
        
        % Follows the Given Trajectory
        function followTrajectory(rc, traj)
            
            if ~traj.data.precomputed
                disp('Precomputing Workspace Trajectory. . .');
                dt = traj.params.tcrit(end) / (500-1); % use 200pts
                traj.precompute(dt);
            end
            
            %% PRECOMPUTE JOINT TRAJECTORY AND INTERP.
            
            disp('Executing Trajectory . . .');
            numCommands = 0; % Number of Times the Robot Trajectory has Been Updated
            first_loop = 1;
            done = 0;
            while (~done)
                if(first_loop)
                    rc.clock = Clock();
                    first_loop = 0;
                end

                T = rc.clock.time();
%                     rc.clock.pause();
%                     rc.clock.resume(); % Break here for in-loop debugging
                
                % Compute Target Robot State:

                rc.moveTo(traj.x_t(T));
                rc.moveAt(traj.v_t(T));
                rc.issueCommand();
                numCommands = numCommands + 1;
                
                if rc.sim
                    % Replot Waypoints Ontop of Trajectory
                    hold on
                        scatter3(traj.points(:,1), traj.points(:,2), traj.points(:,3));
                    hold off
                end
    
                if(T > traj.data.ts(end))
                    done = 1;
                    % Stop Immediately:
                    rc.moveTo(traj.data.xs(end));
                    rc.command.v = rc.robot.zeros;
                    rc.command.sent = false;
                    rc.issueCommand();
                else
                    pause(0.025); % CPU Relief
                end
            end % ~done?
            
            finTime = rc.clock.time();
            disp('Trajectory Completed');
            disp('Avg. Cycle Time / Latency: %f', finTime / numCommands);
        end % #followTrajectory
        
        % Sends the Current Command to the Robot
        function issueCommand(rc)
            if ~rc.command.sent
                if rc.sim
                    if ~isempty(rc.command.q)
                        rc.robot.visualize(q);
                    end
                else
                    warning('#issueCommand Not Implemented for Robot Hardware');
                    % @TODO
                end
                
                rc.command.sent = true;
            end
        end % #issueCommand
        
        % Sets the Current Command to Move the Robot to the Given Workspace
        % Position X.
        % NOTE: This DOES NOT issue the command; #issueCommand MUST be
        % called for the robot to move.
        function moveTo(rc, X)
            if isempty(rc.currState.q)
                rc.updateState(); % This must be the first command issued
            end
            rc.command.q = rc.robot.ik(rc.currState.q, X);
            rc.command.sent = false;
        end
        
        % Sets the Current Command to Move the Robot to the Given Workspace
        % Velocity V.
        % NOTE: This DOES NOT issue the command; #issueCommand MUST be
        % called for the robot to move.
        function moveAt(rc, V)
            if isempty(rc.command.q)
                J = rc.robot.jacobianOf(rc.robot.dof, rc.command.q);
                rc.command.v = pinv(J)*V;
                rc.command.sent = false;
            else
                warning('Position Command Must be Issued Before Workspace Velocity can be Transformed to Joint Velocity');
            end
        end
        
        % Sets the Current Command to Move the Robot with the Given
        % Workspace Force Vector, F.
        % NOTE: This DOES NOT issue the command; #issueCommand MUST be
        % called for the robot to move.
        function moveWith(rc, F)
            if isempty(rc.command.q)
                J = rc.robot.jacobianOf(rc.robot.dof, rc.command.q);
                rc.command.t = J'*F;
                rc.command.sent = false;
            else
                warning('Position Command Must be Issued Before Workspace Force can be Transformed to Joint Forces');
            end
        end
        
        % Sets Whether This is Controlling a VR Simulatio or Not
        function isSim(rc, s)
            rc.sim = s;
            if ~s
                rc.initializeHardwareConnection();
            end
        end % #isSim
    end % methods
    
    
end % classdef RobotController