function [] = welding_trajectories()
% feeding_trajectories

%% Clear out old information to reduce problems with stale modules
HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
HebiLookup.clearGroups();
pause(3);

%% Connect to physical robot
robotHardware = HebiLookup.newGroupFromNames('Robot B',{'J1','J2', 'J3', 'J4', 'J5'});
robotHardware.setCommandLifetime(2);

warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');
gains = load('capstone_gains.mat');
robotHardware.set('gains', gains.gains);
%% Get initial position
fbk = robotHardware.getNextFeedback();
initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robotHardware.startLog('file', fullfile(currentDir, 'robot_data'));

%% command frequency, in Hz
frequency = 100;

%% DH Parameters definition to be defined by student
dh_parameters = zeros(5,4);
%% create object of Robot3D class
robot = Robot3d(dh_parameters); %Robot3d is the class defined for 3D robot

%% Move the robot to Home Position
disp('');
input('Put the robot in a safe position...','s');
% trajectory = homePositioning(robotHardware, frequency);
%% Load and Define waypoints and time stamps for your trajectory
waypoints =  [];% waypoints of trajectories to be defined by students. Think about important points
times = [];% time series to be defined by students

%% Run the trajectory generation and command trajectory
while (true)
    % --------------- BEGIN STUDENT SECTION ----------------------------------
    
    % --------------- END STUDENT SECTION ------------------------------------
end

% homePositioning(robotHardware, frequency);

%% Stop logging, and plot results
robotHardware.stopLog();

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

end
