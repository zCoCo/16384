function [] = sample_arm(logName)

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

%% Prepare plots
if showGroundTruth
    figure();
    suptitle('Robot Display RR')
    subplot(1,2,1);
    actualVisualizer = RobotVisualizer2D(4, 0.1, false);
    title('Your Kinematics');
    subplot(1,2,2);
    gtVisualizer = RobotVisualizer2D(4, 0.1, false);
    title('Correct Kinematics');
else
    actualVisualizer = RobotVisualizer2D(4, 0.1, true);
    title('Robot Display RR');
end

robot = Robot([0.55;0.4],[1;1],[1;1],0);

%% Iteratively visualize data
for i = 1:length(theta)
    
    % Actual frames
    frames = zeros(3,3,4);
    frames(:,:,1) = eye(3);
    frames(:,:,2:end) = robot.fk(theta(i,:)');
    actualVisualizer.setFrames(frames);
    
    % Ground truth
    if showGroundTruth
        frames = log.ground_truth_frames{i};
        gtVisualizer.setFrames(frames);
    end
    
    % play through as fast as MATLAB can plot
	drawnow;
    
    % If this is too fast, you can add a pause here
    % pause(0.001);
    
end

end