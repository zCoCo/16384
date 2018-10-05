function [] = repeat_waypoints(robot, waypoints, segment_duration)
% repeat_waypoints
%
%   Smoothly moves to the initial waypoint from the current position, then plays
%   through the specified waypoints with a given duration between waypoints.
%
%   Linearly interpolates waypoints in joint angle space.
%
%   'robot' is the Robot object to use for any required FK/IK calculations.
%
%   'waypoints' is a matrix of joint angle waypoints, where each column
%   represents a single waypoint (set of joint angles)
%
%   'segment_duration' is the time (in seconds) that the robot should take to
%   move between each pair of waypoints.


% Connect to physical robot
robot_hardware = HebiLookup.newGroupFromFamily('*');

warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');

%% Setup reusable structures to reduce memory use in loop
cmd = CommandStruct();
fbk = robot_hardware.getNextFeedback();

%% Get initial position
fbk = robot_hardware.getNextFeedback();
initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)

% --------------- BEGIN STUDENT SECTION ----------------------------------

%% You will
%% 1) Compute the joint trajectories from initial_thetas to waypoints(:,1), and
%%    then from each of waypoints(:,i) to waypoints(:,i+1), using appropriate
%%    resolution so that when commanded at 100Hz, these will last for
%%    approximately 'segment_duration' seconds.
n_points = 100 * segment_duration; % commands/sec * sec
trajectories = zeros(size(initial_thetas,1), n_points, size(waypoints,2));
trajectories(:,:,1) = linear_joint_trajectory(initial_thetas, waypoints(:,1), n_points);
i = 2;
while(i <= size(waypoints,2))
    trajectories(:,:,i) = linear_joint_trajectory(waypoints(:,i-1), waypoints(:,i), n_points);
    i=i+1
end

%% 2) Start logging (use below code)
currentDir = fileparts(mfilename('fullpath'));
logFile = robot_hardware.startLog('file', fullfile(currentDir, 'repeat_waypoints'));

%% 3) Move through each of these in series, using loops similar to:
%%
%%    for i = 1:num_points_in_trajectory
%%        % Send command to the robot
%%        cmd.position = trajectory(:,i)'; % transpose turns column into row vector for commands
%%        robot_hardware.set(cmd);
%%
%%        % Wait a little bit to send at ~100Hz.
%%        pause(0.01);
%%    end
for i = 1:size(waypoints,2)
    for j = 1:n_points
        % Send command to the robot
        cmd.position = trajectories(:,j,i)'; % transpose turns column into row vector for commands
        robot_hardware.set(cmd);

        % Wait a little bit to send at ~100Hz.
        pause(0.01);
    end
end

% --------------- END STUDENT SECTION ------------------------------------

%% Stop logging, and plot results
robot_hardware.stopLog();

hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'repeat_waypoints.hebilog'));

% Compute workspace end effector path
n = length(hebilog.time);
x = zeros(n,1);
y = zeros(n,1);
for i = 1:n
  ee = robot.end_effector(hebilog.position(i,:)');
  x(i) = ee(1);
  y(i) = ee(2);
end

% Plot path data
figure();
plot(x, y, 'k-', 'LineWidth', 1);
title('Plot of end effector position during trajectory');
xlabel('x [m]');
ylabel('y [m]');
axis equal;

% Plot angle data
figure();
subplot(3,1,1);
plot(hebilog.time, hebilog.position, 'LineWidth', 1)
title('Plot of joint positions during trajectory');
xlabel('t');
ylabel('\theta');
subplot(3,1,2);
plot(hebilog.time, hebilog.velocity, 'LineWidth', 1)
title('Plot of joint velocities during trajectory');
xlabel('t');
ylabel('joint velocities');
subplot(3,1,3);
plot(hebilog.time, hebilog.torque, 'LineWidth', 1)
title('Plot of joint torques during trajectory');
xlabel('t');
ylabel('\tau');
