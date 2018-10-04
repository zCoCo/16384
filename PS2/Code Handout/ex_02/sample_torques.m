function [] = sample_torques(logName)

%% Load log data (allow for sample data as well as arbitrary logs)
if nargin == 0
    % Default to sample log
    log = load('sample_ground_truth.mat');
    theta = log.theta;
    showGroundTruth = true;
else
    % Show torques for arbitrary RR hebi-logs. We do this by loading a MAT file
    % that gets cleaned after load in order to keep the directory clean.
    matFile = HebiUtils.convertGroupLog(logName, 'LogFormat', 'MAT');
    log = load(matFile);
    theta = log.position;
    delete(matFile);
    showGroundTruth = false;    
end


%% Calculate joint torques for (1) applying a desired force and (2) gravity
%% compensation.
n = length(theta);
tauDesiredForce = zeros(n,2);
tauGravComp = zeros(n,2);

% End effector should exert this force [N]
desiredForce = [-1;1];

for i = 1:n
  % Get the joint angle for this timestep
  th = theta(i,:);

  % Get the torques needed to apply an end effector force
  tau = get_joint_torques(th, desiredForce);
  tauDesiredForce(i, 1) = tau(1);
  tauDesiredForce(i, 2) = tau(2);

  % Get the torques needed to compensate for gravity in the
  % negative y direction
  tau = get_grav_comp_torques(th, [0; -9.8]);
  tauGravComp(i, 1) = tau(1);
  tauGravComp(i, 2) = tau(2);
end

%% Plot actual data.

% Torques for desired force
figure();
subplot(2,1,1);
plot(tauDesiredForce(:,1), 'k-', 'LineWidth', 1);
title('Joint 1 torque for end effector to exert force of [-1, 1] N');
xlabel('timestep');
ylabel('Joint torque [Nm]');
subplot(2,1,2);
plot(tauDesiredForce(:,2), 'k-', 'LineWidth', 1);
title('Joint 2 torque for end effector to exert force of [-1, 1] N');
xlabel('timestep');
ylabel('Joint torque [Nm]');

% Plot additional ground truth
if showGroundTruth
    subplot(2,1,1);
    hold on;
    plot(log.tau_desired_force(:,1), 'g--', 'LineWidth', 1);
    hold off;
    legend('Your Joint Torque', 'Correct Joint Torque', ...
           'location', 'northEast');
    subplot(2,1,2);
    hold on;
    plot(log.tau_desired_force(:,2), 'g--', 'LineWidth', 1);
    hold off;
    legend('Your Joint Torque', 'Correct Joint Torque', ...
           'location', 'northEast');
end

% Torques for gravity compensation
figure();
subplot(2,1,1);
plot(tauGravComp(:,1), 'k-', 'LineWidth', 1);
title('Joint 1 torque for gravity compensation');
xlabel('timestep');
ylabel('Joint torque [Nm]');
subplot(2,1,2);
plot(tauGravComp(:,2), 'k-', 'LineWidth', 1);
title('Joint 2 torque for gravity compensation');
xlabel('timestep');
ylabel('Joint torque [Nm]');

% Plot additional ground truth
if showGroundTruth
    subplot(2,1,1);
    hold on;
    plot(log.tau_grav_comp(:,1), 'g--', 'LineWidth', 1);
    hold off;
    legend('Your Joint Torque', 'Correct Joint Torque', ...
           'location', 'northEast');
    subplot(2,1,2);
    hold on;
    plot(log.tau_grav_comp(:,2), 'g--', 'LineWidth', 1);
    hold off;
    legend('Your Joint Torque', 'Correct Joint Torque', ...
           'location', 'northEast');
end

end
