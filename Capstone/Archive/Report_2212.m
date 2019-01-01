function Report_2212()
    % Setup Robot:
    dhp = [...
        0,			pi/2,		116.23e-3,		0;		...
        327.76e-3,	pi,			0,				0;		...
        0,          -pi/2,      2.5e-3,			-pi/2;	...
        254.10e-3,	pi/2,		94.05e-3,		-pi/2;	...
        266.70e-3,	pi/2,		54.23e-3,		0		...
    ];
    actuatedJoints = 4 * ones(size(dhp,1),1);
    robot = Robot3D(dhp, actuatedJoints);
    
    q = [pi/2; 0; 0; pi/2; 0];
    
    N = 30;
    pts = zeros(3,N*N*N);
    
    i = 1;
    ths = linspace(-pi,pi,N);
    tic
    for a = ths
        for b = ths
            for c = ths
                q([2,3,5]) = [a,b,c];
                H = robot.fkf(robot.dof, q);
                pts(:,i) = H(1:3,4);
                i = i+1;
            end
        end
    end
    toc
    
    figure();
    scatter(pts(2,:), pts(3,:), 2);
    xlabel('World Y-Axis');
    ylabel('World Z-Axis');
    grid on
    pause;
    disp('wait');
end % #Report_2212