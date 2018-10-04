function [] = sample_path(logName)

%% Load log data (allow for sample data as well as arbitrary logs)
if nargin == 0
    % Default to sample-log
    log = load('sample_ground_truth.mat');
    theta = log.theta;
    showGroundTruth = true;
else
    % Show path of arbitrary RR hebi-logs. We do this by loading a MAT file
    % that gets cleaned after load in order to keep the directory clean.
    matFile = HebiUtils.convertGroupLog(logName, 'LogFormat', 'MAT');
    log = load(matFile);
    theta = log.position;
    delete(matFile);
    showGroundTruth = false;    
end

%% Calculate end effector path through entire log
n = length(theta);
x = zeros(n,1);
y = zeros(n,1);

% calculate end effector x/y for each timestep
for i = 1:n
    frames = forward_kinematics_RR(theta(i,:));
    x(i) = frames(1, 3, end);
    y(i) = frames(2, 3, end);
end

% Plot actual data
figure();
plot(x, y, 'k-', 'LineWidth', 1);
title('Plot of end effector position over a sample run.');
xlabel('x [m]');
ylabel('y [m]');
axis equal;
d = 1; % Limit the range to +/- 1 meter by default
xlim([-d d]); ylim([-d d]);

% Plot additional ground truth
if showGroundTruth
    hold on;
    plot(log.ground_truth_x, log.ground_truth_y, 'g--', 'LineWidth', 1);
    hold off;
    legend('Your Kinematics', 'Correct Kinematics', 'location', 'southOutside');
end

end