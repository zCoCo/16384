function demo4()
    robotA = Robot([0.3429;0.254;0.2413], zeros(3,1), zeros(3,1), 0);
    robotB = Robot([0.3429;0.2413], zeros(2,1), zeros(2,1), 0);
    robot = robotB;

    wps = teach_waypoints();
    repeat_waypoints(robot, wps, 2);
end % #demo4