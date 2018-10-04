function [] = create_submission()
    % The lab handin directory.
    currentDir = fileparts(mfilename('fullpath'));
    outputFile = fullfile(currentDir, 'handin.tar.gz');

    if exist(outputFile,'file')
        delete(outputFile);
    end

    % Check that submission runs cleanly.  Doesn't check for correctness!
    try
        ex_01;
        clearvars -except outputFile;
    catch
        error('Your code does not run cleanly.  Check ex_01.');
    end

    try
        ex_02;
        clearvars -except outputFile;
    catch
        error('Your code does not run cleanly.  Check ex_02.');
    end

    try
        ex_03([], []);
        clearvars -except outputFile;
    catch
        error('Your code does not run cleanly.  Check ex_03.');
    end

    
    files = {'ex_01.m', 'ex_02.m', 'ex_03.m', 'writeup.pdf'};
    
    tar(outputFile, files);
end