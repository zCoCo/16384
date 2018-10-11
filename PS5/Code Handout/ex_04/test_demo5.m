function test_demo5(using_3dof_robot)
    f = 100;
    times = [0,1,2,3,4];
    wps2 = [...
        [0.62;0.87],...
        [0.06;0.70],...
        [0.43;-0.71],...
        [0.71;0.54],...
        [0.66;1.84]...
    ];
    wps3 = [...
        [0.62;0.87;1.38],...
        [0.06;0.70;0.92],...
        [0.43;-0.71;-0.48],...
        [0.71;0.54;-1.26],...
        [0.66;1.84;-1.83]...
    ];

    if(using_3dof_robot)
        wps = wps3;
    else
        wps = wps2;
    end
    
    c_traj = trajectory_const_vel(wps, times, f);
    t_traj = trajectory_trap_vel(wps, times, f, 0.25);
    s_traj = trajectory_spline(wps, times, f);
    %%
    disp('Playing C Traj, No Vel');
    play_trajectory(c_traj, 0);
    disp('Done. Press to Continue . . .'); pause;
    
    disp('Playing C Traj, With Vel');
    play_trajectory(c_traj, 1);
    disp('Done. Press to Continue . . .'); pause;
    %%
    disp('Playing T Traj, No Vel');
    play_trajectory(t_traj, 0);
    disp('Done. Press to Continue . . .'); pause;
    
    disp('Playing T Traj, With Vel');
    play_trajectory(t_traj, 1);
    disp('Done. Press to Continue . . .'); pause;
    %%
    disp('Playing S Traj, No Vel');
    play_trajectory(s_traj, 0);
    disp('Done. Press to Continue . . .'); pause;
    
    disp('Playing S Traj, With Vel');
    play_trajectory(s_traj, 1);
    disp('Done. Press to Continue . . .'); pause;
end