function CJT_test_sandbox()
    tic
    traj = CJT(csvread('sine.csv'), 1000, 10, 1000, 250);
    toc
    traj.dist(end)
    traj.params
    
    figure();
    traj.plot_pts();
    
    figure();
    traj.plot_t();
    figure();
    traj.plot_s();
    
end % #sandbox