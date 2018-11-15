function [] = create_submission()
    % The lab handin directory.
    currentDir = fileparts(mfilename('fullpath'));
    outputFile = fullfile(currentDir, 'handin-6.tar.gz');

    if exist(outputFile,'file')
        delete(outputFile);
    end

    % Check that submission runs cleanly.  Doesn't check for correctness!
    try
        validate_robot_class();
        clearvars -except outputFile;
    catch
        error('Your code does not run cleanly.  Check again before submission.');
    end
    
    files = {'Robot3D.m', 'validate_robot_class.m', 'poses.fig', 'forces.fig'};
    
    tar(outputFile, files);
end