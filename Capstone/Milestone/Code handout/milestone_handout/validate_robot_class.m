function [] = validate_robot_class(logName)

%% Load log data (allow for sample data as well as arbitrary logs)
if nargin == 0
    % Default to sample-log
    log = load('sample_ground_truth.mat');
    theta = log.theta;
    torques = log.torques;
    dh_parameter = log.dhp;
else
    % Show path of arbitrary RR hebi-logs. We do this by loading a MAT file
    % that gets cleaned after load in order to keep the directory clean.
    matFile = HebiUtils.convertGroupLog(logName, 'LogFormat', 'MAT');
    log = load(matFile);
    theta = log.position;
    delete(matFile);
end

%% Calculate end effector path through entire log
n = length(theta);
pose = zeros(n,6);
forces = zeros(n,6);

%% --------------- BEGIN STUDENT SECTION ----------------------------------
robot = Robot3D(dh_parameter);
for i = 1:size(theta,1) % For each timestep:
% calculate end effector pose
    ths = theta(i,:)';
    pose(i,:) = robot.ee(ths);
    
% calculate end effector forces given joint torques
    Ts = torques(i,:)';
    jacobians = robot.jacobians(ths); % All jacobians
    Je = jacobians(:,:,end); % Jacobian to end effector
    forces(i,:) = -pinv(Je') * Ts;
end

% --------------- END STUDENT SECTION ------------------------------------
%% Plot actual data
labels = {'X\_ee [m]', 'Y\_ee [m]', 'Z\_ee [m]', 'roll [rad]', 'pitch [rad]', 'yaw [rad]'};
figure();
title('Plot of end effector position over a sample run.');
for i =1:6
    subplot(2,3,i);
    hold on;
    plot(log.time, pose(:,i)', 'k-', 'LineWidth', 1);
    plot(log.time, log.pose(:,i), 'g--', 'LineWidth', 1);
    xlabel('time');
    ylabel(labels{i});
end
savefig('poses');
%%
labels = {'Fx\_ee [N]', 'Fy\_ee [N]', 'Fz\_ee [N]', 'Mx\_ee [N-mm]', 'My\_ee [N-mm]', 'Mz\_ee [N-mm]'};
figure();
title('Plot of end effector position over a sample run.');
for i =1:6
    subplot(2,3,i);
    hold on;
    plot(log.time, forces(:,i)', 'k-', 'LineWidth', 1);
    plot(log.time, log.forces(:,i), 'g--', 'LineWidth', 1);
    xlabel('time');
    ylabel(labels{i});
    legend('Calculated', 'Ground Truth');
end
savefig('forces');

end