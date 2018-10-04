function [] = trace_object()
% trace_object
%
%   This function connects to a robot and records the feedback data 
%   during a trace. Note that this reuses the kinematics function from
%   ex_01.
%
%   Make sure that the corresponding robot is plugged in.
%
%   Example:
%      trace_object();
%  

%% Robot Connection
robot = HebiLookup.newGroupFromNames('Robot A', {'J1', 'J2'});

%% Tracing
% Setup visualization
visualizer = RobotVisualizer2D();
disp('Press spacebar to start tracing.');
tracing = false;

% Setup reusable structures to reduce memory use in loop
cmd = CommandStruct();
tmpFbk = robot.getNextFeedback();
while true
    
    %% Read Robot feedback
    fbk = robot.getNextFeedback(tmpFbk);
    cmd.torque = [0 0];
    robot.set(cmd); % command zero torque to improve backdriveability
    
    
    %% Visualize current state
    frames = forward_kinematics_RR(fbk.position);
    visualizer.setFrames(frames);
    drawnow;
    
    
    %% Check keyboard trigger for start/stop conditions
    key = get(gcf,'CurrentCh');
    if strcmp (key , ' ')
        
        if ~tracing
            disp('Tracing started... Press spacebar to stop.');
            currentDir = fileparts(mfilename('fullpath'));
            logFile = robot.startLog('file', fullfile(currentDir, 'star_trace'));
            tracing = true;
        else
            disp('Tracing stopped.');
            robot.stopLog();
            break; % jumps out of while loop
        end
        
        % Move window back into focus and override last set key so that
        % the loop doesn't trigger until next key press.
        figure(gcf);
        set(gcf,'CurrentCh','D');

    end
    
end

%% Path visualization
disp('Visualizing traced object.');
sample_path(logFile);

end