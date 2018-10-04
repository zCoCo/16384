function [] = virtual_spring()
% virtual_spring
%
%   This function makes the robot act as a 'virtual spring', extering joint
%   torques that mimic a force on the end effector towars a set point.
%
%   This function connects to a robot and records the feedback data while
%   executing setting appropriate joint torques.
%
%   Note that this should reuse function(s) written in ex_02.
%
%   Make sure that the corresponding robot is plugged in.
%
%   Example:
%      virtual_spring();
%  

%% Get information about the robot:
robot = robot_info();
% Extract length of the links
l1 = robot.link_lengths(1);
l2 = robot.link_lengths(2);

%% Robot Connection
robot = HebiLookup.newGroupFromNames('Robot B', {'J1', 'J2'});
% Set up handler to be called on exception or ctrl-C.
onCleanup(@() cleanup(robot));

warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
input('Once ready, press "enter" to continue; press "ctrl-c" when finished...','s');

%% Define variables for spring:
% spring constant of the virtual spring
k_spring = 7.5; % [N/m]
% (x,y) end point of the virtual spring; displacement to the
% end effector is measured from here.
center_pt_spring = [0.3; 0.45]; % [m]

%% Setup reusable structures to reduce memory use in loop
cmd = CommandStruct();
tmpFbk = robot.getNextFeedback();

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robot.startLog('file', fullfile(currentDir, 'virtual_spring'));

%% Loop, sending commands to the robot, until 'ctrl-c' is pressed.
while true
    
    % Read Robot feedback
    fbk = robot.getNextFeedback(tmpFbk);
    theta = fbk.position;

    % --------------- BEGIN STUDENT SECTION ----------------------------------
    % Set the torques that the robot should command (should move the end
    % effector towards 'center_pt_spring').
    fkin = forward_kinematics_RR(theta);
    p0 = fkin(:,:,2) * [l2; 0; 1];
    
    x = p0(1:2) - center_pt_spring;
    Fapp = -k_spring * x;
    
    Js = jacobian_link_ends_RR(theta);
    ts = Js(:,:,2)' * [Fapp; 0];
    
    cmd.torque = ts'; % Note: should be a 1x2 (row) vector.

    % --------------- END STUDENT SECTION ------------------------------------

    % Send command to robot; limit velocity to damp out fast motions.
    %cmd.velocity = [0,0];
    robot.set(cmd);

    % Wait a little bit; here we'll cap command rates to 100Hz.
    pause(0.1);
end

function cleanup(robot)

robot.stopLog();

hebilog = HebiUtils.convertGroupLog('virtual_spring.hebilog');

% Check that the three start points were achieved by calculating minimum
% distance from known good angles.
thresh = 0.1; % [rad]
start_pt_1 = [-0.44 1.84];
start_pt_2 = [-0.34 2.46];
start_pt_3 = [ 0.96 1.35];
dist_1 = sqrt(min(sum((hebilog.position - repmat(start_pt_1,size(hebilog.position,1),1)).^2,2)));
dist_2 = sqrt(min(sum((hebilog.position - repmat(start_pt_2,size(hebilog.position,1),1)).^2,2)));
dist_3 = sqrt(min(sum((hebilog.position - repmat(start_pt_3,size(hebilog.position,1),1)).^2,2)));

if (dist_1 > thresh)
    warning('Did not get close enough to start point 1! Please re-run script before submitting!')
end
if (dist_2 > thresh)
    warning('Did not get close enough to start point 2! Please re-run script before submitting!')
end
if (dist_3 > thresh)
    warning('Did not get close enough to start point 3! Please re-run script before submitting!')
end

if (dist_1 <= thresh && dist_2 <= thresh && dist_3 <= thresh)
    disp('Successfully reached all start points, and created log for submission');
end

end

end
