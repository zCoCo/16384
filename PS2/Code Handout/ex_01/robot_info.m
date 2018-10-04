function [ robot_info ] = robot_info()

robot_info = struct();
% length of the links
robot_info.link_lengths = [(12+7/8)*25.4/1000, 9.5*25.4/1000]; % It is your responsibility to measure the links
% masses [kg]
% (density of link tube is .1213 kg/m, with additional .165 kg of hardware at
% ends)
robot_info.link_masses = robot_info.link_lengths * 0.1213 + [.165, .165];
robot_info.joint_masses = [0.347; 0.3];
robot_info.end_effector_mass = 0; % Nothing for the end effector for this lab!

end

