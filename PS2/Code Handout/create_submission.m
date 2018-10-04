function [] = create_submission()
    % The lab handin directory.
    currentDir = fileparts(mfilename('fullpath'));
    outputFile = fullfile(currentDir, 'handin-2.tar.gz');

    if exist(outputFile,'file')
        delete(outputFile);
    end

    % Check that submission runs cleanly.  Doesn't check for correctness!
    try
        forward_kinematics_RR([0;0]);
	forward_kinematics_COM_RR([0;0]);
        jacobian_link_ends_RR([0;0]);
        jacobian_coms_RR([0;0]);
        get_joint_torques([0;0],[0;0]);
        get_grav_comp_torques([0;0],[0;0]);
        clearvars -except outputFile;
    catch
        error('Your code does not run cleanly.  Check ex_01.');
    end
    
    if ~exist('ex_01/sample_path.m', 'file')
        error('Cannot find sample_path.m.  Make sure it is in ./ex_01/!');
    end
    
    if ~exist('ex_01/path.fig', 'file')
        error('Cannot find path.fig.  Make sure it is in ./ex_01/!');
    end
    
    if ~exist('ex_03/virtual_spring.hebilog', 'file')
        error('Cannot find virtual_spring.hebilog.  Make sure to run test_virtual_spring!');
    end
    
    if ~exist('ex_04/grav_comp.hebilog', 'file')
        %error('Cannot find grav_comp.hebilog.  Make sure to run test_grav_comp!');
    end
    
    files = {'ex_01/forward_kinematics_RR.m', 'ex_01/forward_kinematics_COM_RR.m', 'ex_01/jacobian_link_ends_RR.m', 'ex_01/jacobian_coms_RR.m', 'ex_01/sample_path.m', 'ex_01/path.fig', 'ex_02/get_joint_torques.m', 'ex_02/get_grav_comp_torques.m', 'ex_02/torques_ee.fig', 'ex_02/torques_grav_comp.fig', 'ex_03/virtual_spring.m', 'ex_03/virtual_spring.hebilog', 'ex_04/gravity_compensation.m', 'writeup.pdf'};
    
    tar(outputFile, files);
end
