function validate_DHP()
    % Setup Robot:
    dhp = [...
        0,			pi/2,		116.23e-3,		0;		...
        327.76e-3,	pi,			0,				0;		...
        0,          -pi/2,      2.5e-3,			-pi/2;	...
        254.10e-3,	pi/2,		94.05e-3,		-pi/2;	...
        266.70e-3,	pi/2,		54.23e-3,		0
    ];
    actuatedJoints = 4 * ones(size(dhp,1),1);
    robot = Robot3D(dhp, actuatedJoints);
    
    % Fetch Data:
    wps_w = csvread('DH_test_wps_w.csv'); % Workspace Waypoints
    wps_j = csvread('DH_test_wps_j.csv'); % Jointspace Waypoints
    log_w = []; % Log of Workspace Parameters Computed Using the DHP Matrix
    
    % Plot Path:
    for i = 1:size(wps_w, 1)
        X = robot.ee(wps_j(i,:)');
        log_w(end+1, :) = X(1:3);
        
        robot.visualize(wps_j(i,:));
        hold on
            scatter3(wps_w(:,1), wps_w(:,2), wps_w(:,3));
            scatter3(log_w(:,1), log_w(:,2), log_w(:,3));
        hold off
        pause(0.03); % CPU Relief
    end
    
    % Report Findings:
    avgDiff = norm(sum(wps_w - log_w, 1) / size(wps_w, 1));
    fprintf('Average Deviation: %f\n', avgDiff);
    
end % #validate_DHP