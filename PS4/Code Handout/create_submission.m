function [] = create_submission()
    % The lab handin directory.
    currentDir = fileparts(mfilename('fullpath'));
    outputFile = fullfile(currentDir, 'handin-4.tar.gz');

    if exist(outputFile,'file')
        delete(outputFile);
    end

    try
        sample_velocities
        clearvars -except outputFile;
        jacobian_example
        clearvars -except outputFile;
    catch
        error('Your code does not run cleanly.  Check ex_01.');
    end

    try
        robot = Robot([1;1],[1;1],[1;1],1);
        initial_angles = [pi/4; pi/2];
        goal = [-sqrt(2); sqrt(2)];
        final_angles = robot.inverse_kinematics(initial_angles, goal);
        clearvars -except outputFile;
    catch
        error('Your code does not run cleanly.  Ensure that you have added the numerical IK code to Robot.m.');
    end
    
    try
        linear_joint_trajectory([pi/2; -pi/4; -pi/2], [0;0;0], 1000);
        clearvars -except outputFile;
        robot = Robot([1;1],[1;1],[1;1],1);
        traj = linear_workspace_trajectory(robot, [-pi/4; -pi/2], [1.5;0], 1000);
        visualize_trajectory(robot, traj);
        clearvars -except outputFile;
    catch
        error('Your code does not run cleanly.  Check ex_03.');
    end

    if ~exist('Robot.m', 'file')
        error('Cannot find Robot.m.  Make sure it is in the base directory!');
    end

    if ~exist('ex_01/jacobian_example.m', 'file')
        error('Cannot find jacobian_example.m.  Make sure it is in ./ex_01/!');
    end

    if ~exist('ex_03/linear_joint_trajectory.m', 'file')
        error('Cannot find linear_joint_trajectory.m. Make sure it is in ./ex_03/!');
    end
    
    if ~exist('ex_03/linear_workspace_trajectory.m', 'file')
        error('Cannot find linear_workspace_trajectory.m.  Make sure it is in ./ex_03/!');
    end

    if ~exist('ex_04/repeat_waypoints.m', 'file')
        error('Cannot find repeat_waypoints.m.  Make sure it is in ./ex_04/!');
    end
    
    if ~exist('ex_05/play_through_workspace_waypoints.m', 'file')
        error('Cannot find play_through_workspace_waypoints.m.  Make sure it is in ./ex_05/!');
    end

    files = {'Robot.m', 'ex_01/jacobian_example.m', 'ex_03/linear_joint_trajectory.m', 'ex_03/linear_workspace_trajectory.m', 'writeup.pdf'};
    
    tar(outputFile, files);
end
