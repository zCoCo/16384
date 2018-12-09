function sandbox()
    traj = CJT('sine.csv', 10, 10, 10, 0.01);
    traj.dist(end)
    traj.params
    
    figure();
    hold on
        plot(traj.data.ts, traj.data.xs);
        plot(traj.data.ts, traj.data.vs);
        plot(traj.data.ts, traj.data.as);
        %plot(traj.data.ts, traj.data.js);
    hold off
    xlabel('Time [s]');
    legend('Position', 'Velocity', 'Acceleration', 'Jerk');
    
end % #sandbox