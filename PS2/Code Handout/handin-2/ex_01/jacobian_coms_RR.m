function [ jacobians ] = jacobian_coms_RR(theta)
% jacobian_link_ends_RR
%
%   Returns a vector of Jacobian matrices, corresponding
%   to points at the center of mass of each link, given the
%   joint angle positions [rad]. The Jacobians computed here
%   are relative to R^2 only (i.e., no theta term).
%
%   The function returns N 2xN matrices, where N is the
%   number of links (and also the number of joints).
%
%   Each matrix describes the differential relationship between
%   a vector of joint angle velocities and the (x,y) motion of the
%   corresponding point.
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
% Define the Jacobians for the frames below.  Feel free to define helper
% variables.

% Shorthand to Minimize Calculation Time and to Ease Reading:
c1 = cos(theta(1)); s1 = sin(theta(1));
c12 = cos(theta(1)+theta(2)); s12 = sin(theta(1)+theta(2));

J_1 = [-l1*s1/2, 0; l1*c1/2, 0; 1,0];
J_2 = [-l1*s1-l2*s12/2, -l2*s12/2; l1*c1+l2*c12/2, l2*c12/2; 1,1];

% --------------- END STUDENT SECTION ------------------------------------

% Pack into a more readable format. DO NOT CHANGE!
jacobians = cat(3, J_1, J_2);
end
