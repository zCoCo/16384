% Controls a Robot Described by a Given Robot3D Object. This involves
% either managing connection to and control of real hardware (Hebi
% actuators) or running a VisualizeRobotics (VR) simulation.
classdef RobotController < handle
    properties
        robot; %                - Robot3D object being controlled
        robotHardware; %        - Close to the Metal Hebi Controller for Issuing Commands to the Robot
        cmd;
        fbk;
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
            
            rc.cmd = CommandStruct();
            rc.fbk = rc.robotHardware.getNextFeedback();
            
            gains = load('robotA_gains.mat'); %contains gains_struct

            %{
            gains.velocityKi = zeros(1,5);
            gains.velocityKd = 0.25*ones(1,5);
            gains.velocityIClamp = 0.25*ones(1,5);
            gains.torqueKp = 0.25*ones(1,5);
            gains.torqueKi = zeros(1,5);
            gains.torqueKd = 0.001*ones(1,5);
            gains.positionKp = [1 15 2 2 2];
            gains.positionKi = [0.005 1 1 0 0];
            gains.positionKd = [0.005 0 0.03 0 0];
            %}
            % display(gains_struct) will show you other fields you can modify
            rc.robotHardware.set('gains', gains.gains_struct); % note this is the set function, not send
        end % #initializeHardwareConnection
        
        % Updates the Robot's State from Feedback
        function updateState(rc)
            if rc.sim
                if isempty(rc.currState.q)
                    home = rc.robot.model.homeConfiguration;
                    rc.currState.q = [home(:).JointPosition];
                else
                    rc.currState.q = rc.commState.q;
                    rc.currState.v = rc.commState.v;
                    rc.currState.t = rc.commState.t;
                end    
            else
                rc.fbk = rc.robotHardware.getNextFeedback();
                rc.currState.q = rc.fbk.position';
                rc.currState.v = rc.fbk.velocity';
                rc.currState.t = rc.fbk.torque';
                % @TODO
            end
        end % #updateState
        
        % Follows the Given Trajectory
        function followJointTrajectory(rc, traj, traj_w)
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
                rc.command.sent = false;
                rc.command.q = traj.position_t(T);
                %rc.command.v = traj.velocity_t(T);
                rc.issueCommand();
                
                numCommands = numCommands + 1;
                
                if rc.sim
                    % Replot Waypoints On Top of Trajectory
                    hold on
                        scatter3(traj_w.points(:,1), traj_w.points(:,2), traj_w.points(:,3));
                    hold off
                end
    
                if(T > traj.times(end))
                    done = 1;
                    % Stop Immediately:
                    rc.command.q = traj.jointPositions(:,end);
                    rc.command.v = traj.jointVels(:,end);
                    rc.command.sent = false;
                    rc.issueCommand();
                else
                    pause(0.0075); % CPU Relief
                end
            end % ~done?
            
            finTime = rc.clock.time();
            disp('Trajectory Completed');
            fprintf('Avg. Cycle Time / Latency: %f\n', finTime / numCommands);
        end % #followTrajectory
        
        % Sends the Current Command to the Robot
        function issueCommand(rc)
            if ~rc.command.sent
                if rc.sim
                    if ~isempty(rc.command.q)
                        rc.robot.visualize(rc.command.q);
                    end
                else
                    rc.cmd.position = rc.command.q'; % transpose turns column into row vector for commands
                    if ~isempty(rc.command.v) && rc.command.v ~= rc.commState.v
                        rc.cmd.velocity = rc.command.v';
                    end
                    if ~isempty(rc.command.t) && rc.command.t ~= rc.commState.t
                        rc.cmd.effort = rc.command.t';
                    end
                    rc.robotHardware.set(rc.cmd);
                end
                
                rc.command.sent = true;
                rc.commState.q = rc.command.q;
                rc.commState.v = rc.command.v;
                rc.commState.t = rc.command.t;
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