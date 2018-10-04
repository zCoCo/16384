function [ waypoints ] = teach_waypoints()
% teach_waypoints
%
%   Returns a matrix of joint angle waypoints, where each column represents a
%   single waypoint (set of joint angles)

% Connect to robot
robot = HebiLookup.newGroupFromFamily('*');

% Set 'zero torque' mode
robot.setCommandLifetime(0); % Ensure commands never expire!
cmd = CommandStruct();
cmd.torque = zeros(1, robot.getNumModules());
robot.set(cmd);

% Collect waypoints
waypoints = zeros(robot.getNumModules(), 0);

while true
    % Wait for keypress
    res = input('Press enter to collect waypoint, or x and enter to quit.','s');
    
    if (res == 'x')
        break;
    end

    fbk = robot.getNextFeedback();
    waypoints(:, end + 1) = (fbk.position)';
end 

% Ensure we don't keep controlling forever!
robot.setCommandLifetime(.1);
robot.set(cmd);

end
