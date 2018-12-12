function robotVisualizationTest()
    dhpMat = [0   	pi/2	0   	0;
            0.4318	0       0       0
            0.0203	-pi/2	0.15005	0;
            0   	pi/2	0.4318	0;
            0       -pi/2	0   	0;
            0       0       0       0]; % Puma560(R) from matlab sample
    actuatedJoints = 4 * ones(6);
    robot = Robot3D(dhpMat, actuatedJoints);
    
    q = zeros(robot.dof,1);
    tic
    robot.visualize(q);
    toc
    
    pause(0.5);
    wps = csvread('sine.csv');
    hold on
        scatter3(wps(:,1), wps(:,2), wps(:,3));
    hold off
    
    pause(1);
    
    for i = 1:robot.dof
        for t = linspace(0,2*pi, 50)
            q(i) = t;
            robot.visualize(q);
            hold on
                scatter3(wps(:,1), wps(:,2), wps(:,3));
            hold off
            pause(0.01);
        end
    end
    
    pause(1);
    
   
    for t = linspace(0,2*pi, 50)
        q(:) = t;
        robot.visualize(q);
        hold on
            scatter3(wps(:,1), wps(:,2), wps(:,3));
        hold off
        pause(0.01);
    end
    
end % #robotVisualizationTest