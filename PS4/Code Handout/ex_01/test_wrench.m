% Returns the Joint Torques Necessary to Produce the Given Wrench
% for a Planar Serial Revolute Robot with n Joints and link lenghts equal
% to 5.
function torques = test_wrench(n,w)
    robot = Robot(5*ones(n,1), zeros(n,1), zeros(n,1), 0);
    thetas = [0;0.1;0.2;0.3;0.4;0;-0.1;-0.2;-0.3;-0.4];
    Js = robot.jacobians(thetas);
    torques = Js(:,:,end)' * w;
end