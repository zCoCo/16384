function validate_gravComp()
    % Initialize Robot and Controller:
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
        [-0.9*pi/2; 0; -pi/2; -pi/2; -pi/2], [0.9*pi/2; pi; pi/2; pi/2; pi/2], ...
        [0;0.11634;0;0.11634;0], ...
        [0;0;0.5120;0.5120;0.5120], ...
        0.075, ...
        [0,0,0,0; 0,0,0,0; 52.50, -53.55, 58, 35]*1e-3, ...
        [0, -164.13, 0, 0; 0,0,0,-126.8; 0,-88.55,0,94.05]*1e-3, ...
        [-123.31; 0; 0]*1e-3 ...
    );
    rc = RobotController(robot);
        rc.isSim(true);
        
    if rc.sim
        rc.command.sent = false;
        rc.command.q = [0.4202; 0.5585; 0.9409;0.1025;0.3524];
        rc.issueCommand();
    end
    rc.updateState();
    
    robot.visualize(rc.currState.q);
    hold on
        robot.plotCOMS(rc.currState.q);
    hold off
        
    % Hold Steady at Current Location:
    while(1)
        rc.updateState();
        rc.command.q = rc.currState.q;
        rc.command.v = rc.currState.v;
        %rc.command.t = robot.getGravComp(rc.currState.q);
        pause(0.01); % CPU Relief
    end
end % #validate_gravComp