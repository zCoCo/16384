function [] = visualize_trajectory(robot, trajectory)

%% Calculate end effector path through entire log
n = size(trajectory,2);
x = zeros(n,1);
y = zeros(n,1);
for i = 1:n
  ee = robot.end_effector(trajectory(:,i));
  x(i) = ee(1);
  y(i) = ee(2);
end

% Plot path data
figure();
plot(x, y, 'k-', 'LineWidth', 1);
title('Plot of end effector position during trajectory');
xlabel('x [m]');
ylabel('y [m]');
axis equal;

% Plot angle data
figure();
plot(1:size(trajectory,2), trajectory, 'LineWidth', 1)
title('Plot of joint positions during trajectory');
xlabel('t');
ylabel('\theta values');

end
