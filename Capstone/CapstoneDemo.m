function CapstoneDemo()
   traj_file = 'sine.csv';
   
    %% Initialize Robot and Controller:
    disp('Initializing Robot and Controller. . .'), beep;
    % Geometry:
    dhp = [...
        0,			pi/2,		116.23e-3,		0;		...
        327.76e-3,	pi,			0,				0;		...
        0,          -pi/2,      2.5e-3,			-pi/2;	...
        270.0e-3,	pi/2,		94.05e-3,		-pi/2;	...
        266.70e-3,	pi/2,		54.23e-3,		0
    ];
    actuatedJoints = 4 * ones(size(dhp,1),1);
    
    % Dynamics:
    gravity = -[0,0,9.81];%-[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)] % Get gravity vector to Compensate for being un-level (i/a)
    
    robot = Robot3D(dhp, actuatedJoints, gravity);
    rc = RobotController(robot);
        rc.sim = true; % Still Simulating...
    
    %% Get the Robot to Hold Still While Doing Computations:
    disp('Balancing Robot. . .'), beep;
    rc.updateState(); % Fetch Feedback Data
    rc.command.sent = false;
    rc.command.q = robot.zeros();
    rc.command.v = robot.zeros();
    rc.command.t = robot.gravComp(rc.currState.q);
    rc.issueCommand();
    
    %% Load and Precompute Traj:
    % Compute Workspace Trajectory from Waypoint File:
    disp('Precomputing Workspace Trajectory . . .'), beep;
    wps = csvread(traj_file); % Get Waypoints from file
    % Prepend Current Position from Feedback for Continuity and "warm start" to IK:
    rc.updateState();
    x0 = robot.ee(rc.currState.q);
    wps = [x0(1:3); x0(1:3); x0(1:3); wps];
    % Initialize Workspace Trajectory:
    traj_w = CJT(wps, 1000, 10, 1000, 250);
    
    % Compute Joint Trajectory from WorkspaceTrajectory:
    disp('Precomputing Jointspace Trajectory. . .'), beep;
    traj_j = JointTraj(traj_w);
    fprintf('  > Target Time: %f\n', traj_w.params.tcrit(end));
    
    %% Display Waypoints, Waypoint Path Normals, Computed Workspace Trajectory, and Current Robot State:
    robot.visualize();
    hold on
        % Waypoints:
        traj_w.plot_pts();
        % Workspace Trajectory (including any interpolation errors):
        function x = p_s_x(s); p = traj_w.point(s); x = p(1); end % sloppy but it works and speed doesn't matter here.
        function y = p_s_y(s); p = traj_w.point(s); y = p(2); end
        function z = p_s_z(s); p = traj_w.point(s); z = p(3); end
        fplot3(p_s_x, p_s_y, p_s_z, [0, traj_w.dist(end)]);
    hold off
    
    %% Wait for Command:
    disp('Ready! Clear the field and press any key to go.'), beep;
    pause
    
    %% Execute Trajectory:
    disp('Executing Trajectory. . .'), beep;
    rc.followTrajectory(traj_j, true);
    
    %% Keep Things Open for Inspection.
    disp('Done!'), beep;
    pause
    
end % #CapstoneDemo