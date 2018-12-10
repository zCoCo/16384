function CJT_test_sandbox()
    tic
    traj = CJT(csvread('sine.csv'), 1000, 10, 2, 250);
    dt = traj.data.timestep;
    toc
    traj.dist(end)
    traj.params
    
%     As = zeros(size(traj.params.tcrit));
%     for(i = 1:length(traj.params.tcrit)-1)
%         traj.params.tcrit(i);
%         j = find(traj.data.ts >= traj.params.tcrit(i));
%         As(i) = sum(traj.data.vs(1:j(1)))*dt;
%     end
%     As(end) = sum(traj.data.vs)*dt;
%     As
%     
%     vp = traj.params.vpeak
%     ap = traj.params.apeak
%     jm = traj.jmax
%     sc = vp*(ap^2 + jm*vp)/(2*ap*jm)
    
    
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
        %plot(traj.data.ts, traj.data.js, 'c');
        %plot(traj.data.ts, ji, 'y');
    hold off
    xlabel('Time [s]');
    legend('Position', 'Velocity', 'Num. Vel.', 'Acceleration', 'Num. Acc.', 'Jerk', 'Num. Jerk');
    
end % #sandbox