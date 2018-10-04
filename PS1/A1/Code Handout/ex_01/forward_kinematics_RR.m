function [ frames ] = forward_kinematics_RR(theta)
% forward_kinematics_RR
%
%   Returns the forward kinematics for an RR robot given
%   the joint angle positions [rad].
%
%   The function returns N 3x3 frames, where N is the
%   combined number of links and joints.
%
%   Each homogeneous transform describes the relationship from
%   the base frame to the corresponding link or joint.
%
%   Hints
%   - 'theta' is a vector. Individual angles can be selected
%      using indices, e.g., theta1 = theta(1)

% length of the links
l1 = 0.375;
l2 = 0.31;

% --------------- BEGIN STUDENT SECTION ----------------------------------
% Define the transforms for all the frames below (replace each 'eye(3)'
% with the corresponding matrix).  You can define helper variable, and
% feel free to express your answers as products of matrices.
% Precompute and store in aliases to make code easier to parse (and marginally faster):
c1 = cos(theta(1)); s1 = sin(theta(1));
c2 = cos(theta(2)); s2 = sin(theta(2));
H_1_0 = [c1, -s1, 0; s1, c1, 0; 0, 0, 1];
H_2_0 = [c1, -s1, l1*c1; s1, c1, l1*s1; 0, 0, 1];
H_3_0 = H_2_0 * [c2, -s2, 0; s2, c2, 0; 0, 0, 1]; % H20 * H32
H_4_0 = H_2_0 * [c2, -s2, l2*c2; s2, c2, l2*s2; 0, 0, 1]; % H20 * H42

% --------------- END STUDENT SECTION ------------------------------------

% Pack into a more readable format. DO NOT CHANGE!
frames = cat(3, H_1_0, H_2_0, H_3_0, H_4_0);
end
