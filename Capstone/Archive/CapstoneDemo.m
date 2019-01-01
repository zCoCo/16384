function CapstoneDemo()
    %% Demo Parameters
   sim = false;
   useGravComp = false;
   traj_file = 'straight.csv';
   if contains(traj_file, 'sine')
        home_position = [0.4202; 0.5585; 0.9409;0.1025;0.3524];
   else
       home_position = [0.5671; 0.3366; 1.469; 0.5551; -0.1556];
   end
   speed = 0.04; % [m/s]
   offset_y = [0,25.4e-3,0]; % Push target points futher away so the arm goes fully through board
   offset_z = [0,0,-6.5e-3]; % Compensate for constant sag (primitive alternative to gravComp if it keeps freaking out).
   
    %% Initialize Robot and Controller:
    disp('Initializing Robot and Controller. . .'), beep;
    % Geometry:
    dhp = [...
        0,			pi/2,		116.23e-3,		0;		...
        327.76e-3,	pi,			0,				0;		...
        0,          -pi/2,      2.5e-3,			-pi/2;	...
        254.10e-3,	pi/2,		94.05e-3,		-pi/2;	...
        266.70e-3,	pi/2,		54.23e-3,		0
    ];
    actuatedJoints = 4 * ones(size(dhp,1),1);
    
    % Dynamics:
    gravity = -[0,0,9.81];%-[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)] % Get gravity vector to Compensate if un-level (i/a)
    
    robot = Robot3D( ...
        dhp, actuatedJoints, ...
        [-0.9*pi/2; 0; -pi/2; -pi/2; -pi/2], [0.9*pi/2; pi; pi/2; pi/2; pi/2] ...
    );
    rc = RobotController(robot);
        rc.isSim(sim);
    
    %% Maneuver to Starting Position:
    disp('Initializing Robot State. . .'), beep;
    if sim
        rc.command.sent = false;
        rc.command.q = home_position;
        rc.issueCommand();
    end
    rc.updateState();
        
    %% Get the Robot to Hold Still While Doing Computations:
    disp('Balancing Robot. . .'), beep;
    if useGravComp
        rc.updateState(); % Fetch Feedback Data
        rc.command.sent = false;
        rc.command.q = rc.currState.q;
        rc.command.v = robot.zero;
        %rc.command.t = robot.gravComp(rc.currState.q);
        rc.issueCommand();
    end
    
    %% Load and Precompute Workspace Trajectory:
    % Compute Workspace Trajectory from Waypoint File:
    disp('Precomputing Workspace Trajectory . . .'), beep;
    wps = csvread(traj_file); % Get Waypoints from file
    
    % Apply Desired Offset to All Points:
    wps = wps + offset_y + offset_z;
    
    % Prepend Current Position from Feedback for Continuity and "warm start" to IK:
    rc.updateState();
    x0 = robot.ee(rc.currState.q);
    A0 = wps(1,:);
    M(:,1) = linspace(x0(1), A0(1)-0.002, 50);
    M(:,2) = linspace(x0(2), A0(2)-0.002, 50);
    M(:,3) = linspace(x0(3), A0(3)-0.002, 50);
    wps = [M; wps];
    % Initialize Workspace Trajectory:
    traj_w = CJT(wps, [0,0,0], 100, 1, speed, 200);
    
    
    %% Display Waypoints, Waypoint Path Normals, Computed Workspace Trajectory, and Current Robot State:
    robot.visualize(rc.currState.q);
    hold on
        % Waypoints, Path Normals, and Path Tangents:
        traj_w.plot_pts();
    hold off
    pause(1); % Give it a second to load
    
    %% Load and Precompute Jointspace Trajectory:
    % Compute Joint Trajectory from WorkspaceTrajectory:
    disp('Precomputing Jointspace Trajectory. . .'), beep;
    traj_j = JointTrajectory(robot, traj_w, rc.currState.q);
    fprintf('  > Target Time: %f\n', traj_w.params.tcrit(end));
    
    %% Plot Jointspace Trajectories
    figure();
    hold on
        for i = 1:robot.dof
            dQdt = zeros(size(traj_j.jointPositions(i,:)));
            dQ = diff(traj_j.jointPositions(i,:));
            dt = diff(traj_j.times);
            dQdt(1:end-1) = dQ ./ dt;
            
            subplot(3,2,i);
            hold on
                plot(traj_j.times, traj_j.jointPositions(i,:), 'b');
                plot(traj_j.times, traj_j.jointVels(i,:), 'r');
                plot(traj_j.times, dQdt, 'c');
                title(strcat('Joint ', num2str(i)));
            hold off
        end
        % Put Legend in Last Slot in Grid:
        subplot(3,2,6);
        hold on
            plot(0, 0, 'b'); plot(0, 0, 'r'); plot(0, 0, 'c');
        hold off
        legend('Joint Position [rad]', 'Joint Velocity [rad/s]', 'Derivative of Joint Position [rad/s]');
    hold off
    
    %% Wait for Command:
    fprintf('Ready! Clear the field and press any key to go.\n'), beep;
    pause
    
    %% Start Logging:
    currentDir = fileparts(mfilename('fullpath'));
    logFile = rc.robotHardware.startLog('file', fullfile(currentDir, 'robot_data'));
    
    %% Execute Trajectory:
    disp('Executing Trajectory. . .'), beep;
    rc.followJointTrajectory(traj_j, traj_w);
    
    %% Stop Logging and Plot Results
    rc.robotHardware.stopLog();

    figure();
    hold on
        for i = 1:robot.dof
            subplot(3,2,i);
            hold on
                plot(rc.log.times, rc.log.errors(i,:)', 'b');
                title(strcat('Joint ', num2str(i)));
            hold off
        end
        % Put Legend in Last Slot in Grid:
        subplot(3,2,6);
        hold on
            plot(0, 0, 'b');
        hold off
        legend('Joint Position Error [rad]');
    hold off
    
    hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));

    % Plot angle data
    figure();
    subplot(3,1,1);
    plot(hebilog.time, hebilog.positionCmd, 'k', 'LineWidth', 1)
    hold on;
    plot(hebilog.time, hebilog.position, 'r--', 'LineWidth', 1)
    hold off;
    title('Plot of joint positions during trajectory');
    xlabel('t');
    ylabel('\theta');
    subplot(3,1,2);
    plot(hebilog.time, hebilog.velocityCmd, 'k', 'LineWidth', 1)
    hold on;
    plot(hebilog.time, hebilog.velocity, 'r--', 'LineWidth', 1)
    hold off;
    title('Plot of joint velocities during trajectory');
    xlabel('t');
    ylabel('joint velocities');
    subplot(3,1,3);
    plot(hebilog.time, hebilog.torque, 'r--', 'LineWidth', 1)
    title('Plot of joint torques during trajectory');
    xlabel('t');
    ylabel('\tau');
    
    %% Keep Things Open for Inspection.
    disp('Done!'), beep;
    pause
    
end % #CapstoneDemo