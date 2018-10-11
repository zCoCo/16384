function [] = create_submission()
    % The lab handin directory.
    currentDir = fileparts(mfilename('fullpath'));
    outputFile = fullfile(currentDir, 'handin-5.tar.gz');

    if exist(outputFile,'file')
        delete(outputFile);
    end

    try
        validate_trajectory_const_vel
        clearvars -except outputFile;
    catch
        error('Your code does not run cleanly.  Check ex_01.');
    end
    try
        validate_trajectory_trap_vel
        clearvars -except outputFile;
    catch
        error('Your code does not run cleanly.  Check ex_02.');
    end
    try
        validate_trajectory_spline
        clearvars -except outputFile;
    catch
        error('Your code does not run cleanly.  Check ex_03.');
    end

    files = {'ex_01/validate_trajectory_const_vel.m', 'ex_02/validate_trajectory_trap_vel.m', 'ex_03/validate_trajectory_spline.m'};
    tar(outputFile, files);
end
