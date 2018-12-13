function CapstoneDemo()
   sim = true; % Still Simulating...
   traj_file = 'sine.csv';
   home_position = [0.4202; 0.5585; 0.9409;0.1025;0.3524];\
   speed = 0.005; % [m/s]
   
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
    gravity = -[0,0,9.81];%-[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)] % Get gravity vector to Compensate for being un-level (i/a)
    
    robot = Robot3D(dhp, actuatedJoints);
    rc = RobotController(robot);
        rc.isSim(sim);
    
    %% Maneuver to Starting Position:
    %{
    disp('Initializing Robot State. . .'), beep;
    rc.command.sent = false;
    rc.command.q = safe_position;
    rc.issueCommand();
    pause(1.5);
    rc.command.sent = false;
    rc.command.q = home_position;
    rc.issueCommand();
        
    %% Get the Robot to Hold Still While Doing Computations:
    disp('Balancing Robot. . .'), beep;
    rc.updateState(); % Fetch Feedback Data
    rc.command.sent = false;
    rc.command.q = robot.zero;
    rc.command.v = robot.zero;
    %rc.command.t = robot.gravComp(rc.currState.q);
    rc.issueCommand();
    %}
    
    %% Load and Precompute Traj:
    % Compute Workspace Trajectory from Waypoint File:
    disp('Precomputing Workspace Trajectory . . .'), beep;
    wps = csvread(traj_file); % Get Waypoints from file
    
    % Prepend Current Position from Feedback for Continuity and "warm start" to IK:
    rc.updateState();
    x0 = robot.ee(rc.currState.q);
    x1 = robot.ee(home_position);
    A0 = wps(1,:);
    M(:,1) = linspace(x0(1), x1(1), 50);
    M(:,2) = linspace(x0(2), x1(2), 50);
    M(:,3) = linspace(x0(3), x1(3), 50);
    N(:,1) = linspace(x1(1)+0.001, A0(1)-0.001, 50);
    N(:,2) = linspace(x1(2)+0.001, A0(2)-0.001, 50);
    N(:,3) = linspace(x1(3)+0.001, A0(3)-0.001, 50);
    wps = [M; N; wps];
    % Initialize Workspace Trajectory:
    traj_w = CJT(wps, [0,0,0], 100, 1, speed, 750);
    
    % Compute Joint Trajectory from WorkspaceTrajectory:
    disp('Precomputing Jointspace Trajectory. . .'), beep;
    traj_j = JointTrajectory(robot, traj_w, rc.currState.q);
    fprintf('  > Target Time: %f\n', traj_w.params.tcrit(end));
    
    %% Display Waypoints, Waypoint Path Normals, Computed Workspace Trajectory, and Current Robot State:
    robot.visualize(rc.currState.q);
    hold on
        % Waypoints:
        traj_w.plot_pts();
        % Workspace Trajectory (including any interpolation errors):
        function x = p_s_x(s); p = traj_w.point(s); x = p(1); end % sloppy but it works and speed doesn't matter here.
        function y = p_s_y(s); p = traj_w.point(s); y = p(2); end
        function z = p_s_z(s); p = traj_w.point(s); z = p(3); end
        fplot3(@p_s_x, @p_s_y, @p_s_z, [0, traj_w.dist(end)]);
    hold off
    
    %% Plot Jointspace Trajectories
    %{
figure();
    for i = 1:robot.dof
        subplot(3,2,i);
        hold on
            plot(traj_j.times, traj_j.jointPositions(:,i), 'b');
            plot(traj_j.times, traj_j.jointVels(:,i), 'r');
        hold off
        title(strcat('Joint ', num2str(i)));
    end
    %}
    
    %% Wait for Command:
    fprintf('Ready! Clear the field and press any key to go.\n'), beep;
    pause
    
    %% Execute Trajectory:
    disp('Executing Trajectory. . .'), beep;
    rc.followJointTrajectory(traj_j, traj_w);
    
    %% Keep Things Open for Inspection.
    disp('Done!'), beep;
    pause
    
end % #CapstoneDemo