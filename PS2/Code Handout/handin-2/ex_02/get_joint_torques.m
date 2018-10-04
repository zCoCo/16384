function [ torque ] = get_joint_torques(theta, desiredForce)
% get_joint_torques
%
%   Calculates the joint torques required to result in a desired force
%   vector (in world coordinates).

% --------------- BEGIN STUDENT SECTION ----------------------------------
% Use the Jacobian to find the joint torques necessary for the end
% effector to exert the given force (given the joint configuration theta).
% Assume 'desiredForce' is a 2x1 (column) vector.
Js = jacobian_link_ends_RR(theta);
ts = Js(:,:,2)' * [desiredForce; 0];
torque1 = ts(1);
torque2 = ts(2);

% --------------- END STUDENT SECTION ------------------------------------
% Pack into a more readable format. DO NOT CHANGE!
torque = cat(1, torque1, torque2);
end
