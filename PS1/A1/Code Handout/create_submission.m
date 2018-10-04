function [] = create_submission()
    % The lab handin directory.
    currentDir = fileparts(mfilename('fullpath'));
    outputFile = fullfile(currentDir, 'handin-1.tar.gz');

    if exist(outputFile,'file')
        delete(outputFile);
    end

    % Check that submission runs cleanly.  Doesn't check for correctness!
    try
        forward_kinematics_RR([0,0]);
        clearvars -except outputFile;
    catch
        error('Your code does not run cleanly.  Check ex_01.');
    end
    
    if ~exist('ex_02/workspace_analysis_PR.m', 'file')
        error('Cannot find workspace_analysis_PR.m.  Make sure it is in ./ex_02/!');
    end
    
    if ~exist('ex_02/workspace_pr_1.fig', 'file')
        error('Cannot find workspace_pr_1.fig.  Make sure it is in ./ex_02/!');
    end
    
    if ~exist('ex_02/workspace_pr_2.fig', 'file')
        error('Cannot find workspace_pr_2.fig.  Make sure it is in ./ex_02/!');
    end
    
    if ~exist('ex_03/star_trace.hebilog', 'file')
        error('Cannot find star_trace.hebilog.  Make sure to run trace_object!');
    end
    
    files = {'ex_01/forward_kinematics_RR.m', 'ex_02/workspace_analysis_PR.m', 'ex_02/workspace_pr_1.fig', 'ex_02/workspace_pr_2.fig', 'ex_03/star_trace.fig', 'ex_03/star_trace.hebilog', 'writeup.pdf'};
    
    tar(outputFile, files);
end