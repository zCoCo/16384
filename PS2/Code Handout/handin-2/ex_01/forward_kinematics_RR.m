function [ frames ] = forward_kinematics_RR(theta)
% forward_kinematics_RR
%
%   Returns the forward kinematics for an RR robot given
%   the joint angle positions [rad].
%
%   The function returns N 3x3 frames, where N is the
%   number of links.
%
%   Each homogeneous transform describes the relationship from
%   the base frame to start of the corresponding link.
%
%   Hints
%   - 'theta' is a vector. Individual angles can be selected
%      using indices, e.g., theta1 = theta(1)

% Get information about the robot:
robot = robot_info();
% Extract length of the links
l1 = robot.link_lengths(1);
l2 = robot.link_lengths(2);

% --------------- BEGIN STUDENT SECTION ----------------------------------
% Define the transforms for all the frames below (replace each 'eye(3)'
% with the corresponding matrix).  You can define helper variables, and
% feel free to express your answers as products of matrices.

% Shorthand to Minimize Calculation Time and to Ease Reading:
c1 = cos(theta(1)); s1 = sin(theta(1));
c2 = cos(theta(2)); s2 = sin(theta(2));

H_1_0 = [c1, -s1, 0; s1, c1, 0; 0,0,1];
H_2_1 = [c2, -s2, l1; s2, c2, 0; 0,0,1]; % translate then rotate
H_2_0 = H_1_0 * H_2_1;

% --------------- END STUDENT SECTION ------------------------------------

% Pack into a more readable format. DO NOT CHANGE!
frames = cat(3, H_1_0, H_2_0);
end
