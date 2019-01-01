function pathTest()
    p = zeros(3,1);
    n = [0;1;0];
    L = 10; %extract me from dhp
    o0 = zeros(3,1); % Origin of the robot
    o5a = p - L*n;
    o5b = p + L*n;
    o5 = o5a;
    if(norm(o5a-o0) > norm(o5b-o0))
        % Pick Direction of Normal which Brings o5 Closest to Origin:
        n = -n;
        o5 = o5b;
    end
    pitch = asin(n(3));
    target = [o5; 0; pitch; 0];
    thetas = robot.ikd(th0, target, 5, [1,2,3,5]);
end