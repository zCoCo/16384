function CJT_test_sandbox()
    dt = 0.01;
    traj = CJT(csvread('sine.csv'), 10, 10, 10, dt);
    traj.dist(end)
    traj.params
    
    % Compute Numerical Derivatives:
    vi = zeros(size(traj.data.xs));
    vi(2:end) = diff(traj.data.xs) / dt;
    vi(1) = vi(2);
    ai = zeros(size(traj.data.vs));
    ai(2:end) = diff(traj.data.vs)/dt;
    ai(1) = ai(2);
    ji = zeros(size(traj.data.as));
    ji(2:end) = diff(traj.data.as)/dt;
    ji(1) = ji(2);
    
    figure();
    hold on
        plot(traj.data.ts, traj.data.xs, 'r');
        plot(traj.data.ts, traj.data.vs, 'g');
        plot(traj.data.ts, vi, 'b');
        plot(traj.data.ts, traj.data.as, 'k');
        plot(traj.data.ts, ai, 'm');
        plot(traj.data.ts, traj.data.js, 'c');
        plot(traj.data.ts, ji, 'y');
    hold off
    xlabel('Time [s]');
    legend('Position', 'Velocity', 'Num. Vel.', 'Acceleration', 'Num. Acc.', 'Jerk', 'Num. Jerk');
    
end % #sandbox