function demo5()
    robotA = Robot([0.3429;0.254;0.2413], zeros(3,1), zeros(3,1), 0);
    robotB = Robot([0.3429;0.2413], zeros(2,1), zeros(2,1), 0);
    robot = robotA;
    
    
    wps = teach_waypoints();
    wpsw = zeros(2, size(wps,2));
    for i = 1:size(wps,2)
        pi = robot.ee(wps(:,i));
        wpsw(:,i) = pi(1:2);
    end
    wpsw = wpsw;
    play_through_workspace_waypoints(robot, wpsw, 2);
end % #demo4