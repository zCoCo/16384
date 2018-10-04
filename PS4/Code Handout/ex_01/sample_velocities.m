function [] = sample_path()

% Default to sample-log
hebilog = load('sample_ground_truth.mat');
time = hebilog.time;
theta = hebilog.theta;
theta_dot = hebilog.theta_dot;

% Create robot (matching sample data)
robot = Robot([1.5;2.5],[1.5;2.5],[1;1],1);

%% Calculate end effector path through entire log
n = length(theta);
x = zeros(n,1);
y = zeros(n,1);
vx = zeros(n,1);
vy = zeros(n,1);

% calculate end effector x/y position and velocity for each timestep
for i = 1:n
    frames = robot.forward_kinematics(theta(i,:)');
    x(i) = frames(1, 3, end);
    y(i) = frames(2, 3, end);
    jacobian = robot.jacobians(theta(i,:)');
    vel = jacobian(:, :, end) * theta_dot(i,:)';
    vx(i) = vel(1);
    vy(i) = vel(2);
end

% 'quiver' is used to plot arrows along each x/y path, and
% the arrows are centered on the x/y position.

% Sub-sample the data (resolution is how many points to skip so arrows are not
% drawn too densely)
subsample_resolution = 10;
x_sub = x(1:subsample_resolution:end);
y_sub = y(1:subsample_resolution:end);
vx_sub = vx(1:subsample_resolution:end);
vy_sub = vy(1:subsample_resolution:end);

% Plot path data and ground truth
figure();
plot(x, y, 'k-', 'LineWidth', 1);
title('Plot of end effector position and velocity over a sample run.');
xlabel('x [m]');
ylabel('y [m]');
hold on;
plot(hebilog.ground_truth_x, hebilog.ground_truth_y, 'g--', 'LineWidth', 1);
quiver(x_sub, ...
       y_sub, ...
       vx_sub, vy_sub, ...
       0.5, 'Color', [0, 0, 0]);
hold off;
legend('Your Kinematics', 'Correct Kinematics', 'location', 'southOutside');

% And independent velocity:
figure();
title('Plot of end effector position and velocity over a sample run.');
subplot(2,1,1);
plot(time, vx, 'k-', 'LineWidth', 1);
xlabel('t');
ylabel('v_x [m/s]');
hold on;
plot(time, hebilog.ground_truth_vx, 'g--', 'LineWidth', 1);
hold off;
legend('Your Velocity', 'Correct Velocity', 'location', 'southEast');
subplot(2,1,2);
plot(time, vy, 'k-', 'LineWidth', 1);
xlabel('t');
ylabel('v_y [m/s]');
hold on;
plot(time, hebilog.ground_truth_vy, 'g--', 'LineWidth', 1);
hold off;
legend('Your Velocity', 'Correct Velocity', 'location', 'southEast');

%axis equal;

end
