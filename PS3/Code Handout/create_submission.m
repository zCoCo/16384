function [] = create_submission()
    % The lab handin directory.
    currentDir = fileparts(mfilename('fullpath'));
    outputFile = fullfile(currentDir, 'handin-3.tar.gz');

    if exist(outputFile,'file')
        delete(outputFile);
    end

    files = {'Robot.m','writeup.pdf'};
    
    tar(outputFile, files);
end