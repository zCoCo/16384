function torques = jacobian_example()
    robot = Robot(5*ones(10,1), zeros(10,1), zeros(10,1), 0);
    thetas = [0;0.1;0.2;0.3;0.4;0;-0.1;-0.2;-0.3;-0.4];
    Js = robot.jacobians(thetas);
    torques = Js(:,:,end)' * [0;5;2];
end