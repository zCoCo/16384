function Report_2213()
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
    
    N = 50;
    pts = zeros(3,N*N*N);
    ws = zeros(1,N*N*N); % Manipulabilities for Each Point
    
    i = 1;
    ths = linspace(-pi,pi,N);
    tic
    for a = ths
        for b = ths
            for c = ths
                q([2,3,5]) = [a,b,c];
                H = robot.fkf(robot.dof, q);
                pts(:,i) = H(1:3,4);
                
                J = robot.jacobianOf(robot.dof, q);
                ws(i) = prod(svd(J));
                
                i = i+1;
            end
        end
    end
    toc
   
    fprintf('Plane of Points Exist on x = %f\n', pts(1,1));
    
    figure();
    %scatter3(pts(2,:), pts(3,:), ws);
    tri = delaunay(pts(2,:), pts(3,:));
    trisurf(tri, pts(2,:), pts(3,:), ws);
    axis vis3d
    shading interp
    %srf.EdgeColor = 'none';
    xlabel('World Y-Axis');
    ylabel('World Z-Axis');
    zlabel('Yoshikawa Manipulability');
    grid on
    colorbar
end % #Report_2213