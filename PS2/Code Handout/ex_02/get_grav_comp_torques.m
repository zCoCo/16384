function [ torque ] = get_grav_comp_torques(theta, gravity)
% get_grav_comp_torques
%
%   Calculates the joint torques required to cancel out effects due to
%   gravity.

% Get information about the robot:
robot = robot_info();
% Extract mass of the links, joint, and end effector [kg]
m_link_1 = robot.link_masses(1);
m_link_2 = robot.link_masses(2);
% m_joint_1 = robot.joint_masses(1); % not necessary
m_joint_2 = robot.joint_masses(2);
m_end_effector = robot.end_effector_mass;

% --------------- BEGIN STUDENT SECTION ----------------------------------
% Use the Jacobian to calculate the joint torques to compensate for the
% weights of the joints, links, and end effector (assuming the acceleration
% due to gravity is given be 'gravity', and it is a 2x1 (column) vector).
Js = jacobian_link_ends_RR(theta);
JCOMS = jacobian_coms_RR(theta);

ts_COM1 = JCOMS(:,:,1)' * -m_link_1 * [gravity; 0];
ts_COM2 = JCOMS(:,:,2)' * -m_link_2 * [gravity; 0];
ts_J2 = Js(:,:,1)' * -m_joint_2 * [gravity; 0];
t_eff = Js(:,:,2)' * -m_end_effector * [gravity; 0];

ts = ts_COM1 + ts_COM2 + ts_J2 + t_eff;

torque1 = ts(1);
torque2 = ts(2);

% --------------- END STUDENT SECTION ------------------------------------
% Pack into a more readable format. DO NOT CHANGE!
torque = cat(1, torque1, torque2);
end
