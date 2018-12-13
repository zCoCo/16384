% Steer a Robot3D Simulation
function steer()
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
    wps = csvread('sine.csv'); % Workspace Waypoints
    %wps_j = csvread('DH_test_wps_j.csv'); % Jointspace Waypoints
    log = []; % Log of Workspace Parameters Computed Using the DHP Matrix
    
    traj = CJT(wps, [0,0,0], 1000, 10, 1000, 500);
    function x = p_s_x(s); pp = traj.point(s); x = pp(1); end % sloppy but it works and speed doesn't matter here.
    function y = p_s_y(s); pp = traj.point(s); y = pp(2); end
    function z = p_s_z(s); pp = traj.point(s); z = pp(3); end
    
    q = [-0.25; pi/3; pi/3; -0.1; 0];%[-70;25;15;90;0;90] * pi/180;
    j = 1;

    function keyPress(~, e)
        switch e.Key
            case '1'
                j = 1;
            case '2'
                j = 2;
            case '3'
                j = 3;
            case '4'
                j = 4;
            case '5'
                j = 5;
            case 'w'
                q(j) = q(j) + 5*pi/180;
            case 's'
                q(j) = q(j) - 5*pi/180;
        end
    end

    firstRun = 1;
    last_q = Inf;
    while(1)
        if sum(q ~= last_q)
            X = robot.ee(q);
            log(end+1, :) = X(1:3);

            fig = robot.visualize(q);

            hold on
                scatter3(wps(:,1), wps(:,2), wps(:,3));
                scatter3(log(:,1), log(:,2), log(:,3));
                % Workspace Trajectory (including any interpolation errors):
                fplot3(@p_s_x, @p_s_y, @p_s_z, [0, traj.dist(end)]);
            hold off


            if firstRun
                fig = figure();
                fig.KeyPressFcn = @keyPress;
                firstRun = 0;
            end
            
            last_q = q;
        end
        pause(0.05); % CPU Relief
    end

end % #steer