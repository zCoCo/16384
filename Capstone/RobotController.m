% Controls a Robot Described by a Given Robot3D Object. This involves
% either managing connection to and control of real hardware (Hebi
% actuators) or running a VisualizeRobotics (VR) simulation.
classdef RobotController < handle
    properties(SetAccess = immutable)
        robot; %                - Robot3D object being controlled
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
        sim = true; % Whether this is controlling an actual robot or a VR sim.
    end % private,public properties
    
    properties(SetAccess = public, GetAccess = public)
        command = struct(... %  - Command to be Sent to the Robot
            'q', [], ... %          - Joint State Vector
            'v', [], ... %          - Joint Velocity Vector
            't', [], ... %          - Joint Torque Vector
            'sent', false ... %     - Whether This Command has been Sent Yet
        );
    end % public,public properties
    
    methods
        % Instantiates a RobotConroller for the Given Robot3D Object
        function rc = RobotController(rob)
            rc.robot = rob;
        end % ctor
        
        % Initializes Connection to Robot Hardware
        function initializeHardwareConnection(rc)
            warning('#initializeHardwareConnection Not Implemented for Robot Hardware');
            % @TODO
        end % #initializeHardwareConnection
        
        % Updates the Robot's State from Feedback
        function updateState(rc)
            if rc.sim
                rc.get
            else
                warning('#updateState Not Implemented for Robot Hardware');
                % @TODO
            end
        end % #updateState
        
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
            end
        end % #issueCommand
        
        % Sets the Current Command to Move the Robot to the Given Workspace
        % Position X.
        function moveTo(rc, X)
            X = rc.robot.ik
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