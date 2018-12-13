function validate_IK_and_Trajectory()
    % Setup Robot:
    dhp = [...
        0,			pi/2,		116.23e-3,		0;		...
        327.76e-3,	pi,			0,				0;		...
        0,          -pi/2,      2.5e-3,			-pi/2;	...
        270.0e-3,	pi/2,		94.05e-3,		-pi/2;	...
        266.70e-3,	pi/2,		54.23e-3,		0
    ];
    actuatedJoints = 4 * ones(size(dhp,1),1);
    robot = Robot3D(dhp, actuatedJoints);
    
    % Fetch Data:
    wps_w = csvread('sine.csv'); % Workspace Waypoints
    %wps_j = csvread('DH_test_wps_j.csv'); % Jointspace Waypoints
    log_w = []; % Log of Workspace Parameters Computed Using the DHP Matrix
    
    traj = CJT(wps_w, [0,0,0], 1000, 10, 1000, 500);
    
    last_q = [-0.25; pi/3; pi/3; -0.1; 0];%wps_j(1,:)'; % Kickstart IK.
    numUpdates = 0; % Number of Times Logging has been updated
    diffSum = 0; % Sum of differences between target and result.
    for s = linspace(0, traj.dist(end), 100)
        % Get Target Workspace Position:
        p = [traj.point(s)'; traj.RPY_s(s, 1,2)'];
        
        % Use IK to Convert Target Position to Joint Configuration:
        q = robot.ikd(last_q, p, robot.dof, [1,2,3]); % we don't care about rotation about the pointer x, yaw, idx 6.
        last_q = q;
        
        % Convert Joint Configuration Back to Workspace Position:
        X = robot.ee(q);
        log_w(end+1, :) = X(1:3);
        
        % Update Data Logging:
        diffSum = diffSum + norm(p-X);
        numUpdates = numUpdates + 1;
        
        robot.visualize(q);
        hold on
            scatter3(wps_w(:,1), wps_w(:,2), wps_w(:,3));
            scatter3(log_w(:,1), log_w(:,2), log_w(:,3));
        hold off
        pause(0.03); % CPU Relief
    end

    % Report Findings:
    avgDiff = diffSum / numUpdates;
    fprintf('Average Deviation: %f\n', avgDiff);
    
    hold on
        % Workspace Trajectory (including any interpolation errors):
        function x = p_s_x(s); pp = traj.point(s); x = pp(1); end % sloppy but it works and speed doesn't matter here.
        function y = p_s_y(s); pp = traj.point(s); y = pp(2); end
        function z = p_s_z(s); pp = traj.point(s); z = pp(3); end
        fplot3(@p_s_x, @p_s_y, @p_s_z, [0, traj.dist(end)]);
    hold off
    
end % #validate_IK_and_Trajectory