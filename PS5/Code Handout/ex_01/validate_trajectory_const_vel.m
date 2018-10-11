function [] = validate_trajectory_const_vel()

%% Load saved data
load('ex_01_ground_truth.mat');

frequency = 100; % control frequency

figure();
num_traj = length(waypoints);
for i = 1:num_traj
    my_traj = trajectory_const_vel(waypoints{i}, times{i}, frequency);
    traj_time = linspace(times{i}(1), times{i}(end), size(my_traj,2));
    % Plot actual data
    subplot(num_traj,1,i);
    plot(traj_time, my_traj, 'k-', 'LineWidth', 1);
    title(['Constant Velocity Joint Trajectory - Sample ' num2str(i)]);
    xlabel('t [s]');
    ylabel('\theta [rad]');
    % Plot additional ground truth
    hold on;
    plot(traj_time, traj{i}, 'g--', 'LineWidth', 1);
    xlim([times{i}(1), times{i}(end)]);
    hold off;
    legend('Your Trajectory', 'Correct Trajectory', 'location', 'northEast');
end

end
