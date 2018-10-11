function [ trajectory ] = trajectory_trap_vel(waypoints, times, frequency, duty_cycle)
% trajectory_const_vel
%
%   Returns a matrix of joint angles, where each column represents a single
%   timestamp. These joint angles form trapezoidal velocity trajectory segments,
%   hitting waypoints(:,i) at times(i).
%
%   'waypoints' is a matrix of waypoints; each column represents a single
%   waypoint in joint space, and each row represents a particular joint.
%
%   'times' is a row vector that indicates the time each of the waypoints should
%   be reached.  The number of columns should equal the number of waypoints, and
%   should be monotonically increasing.  The first number is technically
%   arbitrary, as it serves only as a reference for the remaining timestamps,
%   but '0' is a good default.
%
%   'frequency' is the control frequency which this trajectory should be played
%   at, and therefore the number of columns per second of playback.

% Number of joints:
num_joints = size(waypoints, 1);
% Number of waypoints:
num_waypoints = size(waypoints, 2);
% Number of segments between waypoints:
num_segments = num_waypoints - 1;

if (size(times) ~= [1, num_waypoints])
    error('Size of times vector is incorrect!');
end

if (num_waypoints < 2)
    error('Insufficient number of waypoints.');
end

if (length(frequency) ~= 1 || frequency < 5)
    error('Invalid control frequency (must be at least 5Hz)');
end

if (duty_cycle < 0 || duty_cycle > 0.5)
    error('Invalid duty cycle!');
end

%% First, we go through each segment between waypoints, and calculate how many
%% points we must generate in this segment given our control frequency.
num_points_per_segment = zeros(1, num_segments);
for segment = 1:num_segments
    dt = times(segment+1) - times(segment);
    num_points_per_segment(segment) = dt * frequency;
end

%% Go through each segment of the trajectory, and fill it in segment-by-segment.

% Pre-allocate a trajectory matrix with the correct number of rows and columns.
trajectory = zeros(num_joints, sum(num_points_per_segment));

% The index of the first point in this segment
segment_start_point = 1;
for segment = 1:num_segments
    % Number of points in this segment
    points_in_segment = num_points_per_segment(segment);

    % The index of the last point in this segment
    segment_end_point = segment_start_point + points_in_segment - 1;

    % Find the number of points of the ramp up (and also of the ramp down)
    % section of the trajectory
    num_ramp_points = floor(duty_cycle * points_in_segment);

    % The length of the ramp up time (and also ramp down time), in seconds.
    ramp_time = (times(segment+1) - times(segment)) * duty_cycle;

    % --------------- BEGIN STUDENT SECTION ----------------------------------
    % Find the maximum velocity through this segment, based on the duty cycle
    th0 = waypoints(:, segment); % Starting Angle
    thf = waypoints(:, segment+1); % Goal Angle
    t0 = times(segment);
    tf = times(segment+1);
    vm = (thf-th0) ./ (times(segment+1) - times(segment) - ramp_time);

    i = 0;
    while(i < num_ramp_points)
        t = i / frequency + t0;
        trajectory(:,segment_start_point+i) = th0 + (vm./2./ramp_time) .* (t - t0)^2;
    i = i+1;
    end
    
    tha = th0 + (vm./2./ramp_time) .* (num_ramp_points / frequency)^2;
    ta = num_ramp_points / frequency + t0;
    tb = (points_in_segment - num_ramp_points - 1) / frequency + t0;
    thb = tha + vm.*(tb-ta);
    
    j = 1;
    while( j <= size(th0,1) )
        trajectory(j,segment_start_point+num_ramp_points:segment_end_point-num_ramp_points) = linspace(tha(j), thb(j), points_in_segment-2*num_ramp_points);
        j = j+1;
    end
    
    i = points_in_segment - num_ramp_points;
    while(i<points_in_segment)
        t = i / frequency + t0;
        trajectory(:,segment_start_point+i) = thb + vm/frequency - (vm./2./ramp_time) .* (tf^2 - 2*t*tf + t^2 - ramp_time^2);
    i = i+1;
    end
    % Fill in the points in this segment of the trajectory.  Those points will
    % be:
    % trajectory(:, segment_start_point : segment_start_point + num_ramp_points - 1)              % ramp up
    % trajectory(:, segment_start_point + num_ramp_points : segment_end_point - num_ramp_points)  % constant velocity
    % trajectory(:, segment_end_point - num_ramp_points + 1 : segment_end_points)                 % ramp down

    % NOTE: it will be useful to default some helper variables here to match
    % the equations from the background material, and potentially loop through
    % the trajectory joint-by-joint and point-by-point to fill in each element
    % directly unless you are familiar with MATLAB vector operations.
    
    % --------------- END STUDENT SECTION ------------------------------------

    % Update the starting index as we move to the next segment.
    segment_start_point = segment_start_point + points_in_segment;
end

    % Encapsulate in a pure-transient function to ensure continuity
    % Returns the position of the joint at the given time t for a
    % trapezoidal profile with max velocity vm, which starts at q0,t0, ends
    % at qf,tf, and has duty cycle d.
    % vm and q can be vectors.
    function q = q_vtrap(t, q0,qf, t0,tf, vm, d)
        tr = (tf-t0) * d;
        ta = t0 + tr;
        tb = tf - tr;
        if(t < t0) %Ensure is safe with out of bounds cases first
            q = q0;
        elseif(t > tf)
            q = qf;
        elseif(t <= ta) % up ramp
            q = q0 + (vm./2./tr) .* (t-t0)^2;
        elseif(t <= tb) % const. vel.
            q = q0 + (vm.*tr./2) + vm.*(t-ta);
        else % down ramp
            q = q0 + (vm.*tr./2) + vm.*(tb-ta) + (vm./2./tr) .* (tf^2 - 2*tf*t + t^2 - tr^2);
        end
    end

end
