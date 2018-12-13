function CapstoneDemo()
   traj_file = 'sine.csv';
   
    %% Initialize Robot and Controller:
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
    gravity = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)] % Get gravity vector to Compensate for being un-level (i/a)
    
    robot = Robot3D(dhp, actuatedJoints, gravity);
    rc = RobotController(robot);
        rc.sim = true; % Still Simulating...
    
    %% Get the Robot to Hold Still While Doing Computations:
    disp('Balancing Robot. . .');
    rc.updateState(); % Fetch Feedback Data
    rc.command.sent = false;
    rc.command.q = robot.zeros();
    rc.command.v = robot.zeros();
    rc.command.t = robot.gravComp(rc.currState.q);
    rc.issueCommand();
    
    %% Load and Precompute Traj:
    % Compute Workspace Trajectory from Waypoint File:
    disp('Precomputing Workspace Trajectory . . .');
    traj_w = CJT(csvread(traj_file), 1000, 10, 1000, 250);
    
    % Compute Joint Trajectory from WorkspaceTrajectory:
    disp('Precomputing Jointspace Trajectory. . .');
    traj_j = JointTraj(traj_w);
    
    %% Display Waypoints, Computed Workspace Trajectory, and Current Robot State:
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
    disp('Ready! Clear the field and press any key to go.');
    pause
    
    %% Execute Trajectory:
    disp('Executing Trajectory. . .');
    rc.followTrajectory(traj_j, true);
    
    %% Keep Things Open for Inspection.
    disp('Done!');
    pause
    
end % #CapstoneDemo