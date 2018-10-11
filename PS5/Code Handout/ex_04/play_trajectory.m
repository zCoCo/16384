function [] = play_trajectory(trajectory, use_velocity)
% repeat_waypoints
%
%   Smoothly moves to the start of the trajectory from the current position,
%   then plays through the specified trajectory at 100Hz.
%
%   'trajectory' is a matrix of joint angles, where each column represents a
%    single set of joint angles.
%
%   'use_velocity' should be set to 'true' if you should also command velocity
%   inputs and 'false' otherwise.

% Clear out old information to reduce problems with stale modules
HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
pause(3);

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

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robot_hardware.startLog('file', fullfile(currentDir, 'robot_data'));

% command frequency, in Hz
frequency = 100;


%% Drive the robot from the initial position to the first waypoint over
%% 2.5 seconds.
traj_from_start = trajectory_trap_vel([initial_thetas trajectory(:,1)], [0, 2.5], 100, 0.25);
traj_vel_from_start = diff(traj_from_start,1,2);

for i = 1 : (size(traj_from_start, 2) - 1)
    % Send command to the robot
    cmd.position = traj_from_start(:,i)'; % transpose turns column into row vector for commands
    if (use_velocity)
      cmd.velocity = traj_vel_from_start(:,i)' * frequency; % transpose turns column into row vector for commands
    end
    robot_hardware.set(cmd);

    % Wait a little bit to send at ~100Hz.
    pause(1 / frequency);
end

%% --------------- BEGIN STUDENT SECTION ----------------------------------

% Using similar code to that above, move through the trajectory passed into this
% function.

%% --------------- END STUDENT SECTION ------------------------------------

%% Stop logging, and plot results
robot_hardware.stopLog();

hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));

% Plot angle data
figure();
subplot(3,1,1);
plot(hebilog.time, hebilog.positionCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.position, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint positions during trajectory');
xlabel('t');
ylabel('\theta');
subplot(3,1,2);
plot(hebilog.time, hebilog.velocityCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.velocity, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint velocities during trajectory');
xlabel('t');
ylabel('joint velocities');
subplot(3,1,3);
plot(hebilog.time, hebilog.torque, 'r--', 'LineWidth', 1)
title('Plot of joint torques during trajectory');
xlabel('t');
ylabel('\tau');
