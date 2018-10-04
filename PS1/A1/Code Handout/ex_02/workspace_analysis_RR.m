function [ ] = workspace_analysis_RR(linkLengths)
% workspace_analysis_RR
%
%   This function visualizes the workspace for a RR configuration.
%   Use this as a basis for developing your RPR analysis.

%%
% Setup parameters
nSamples = 50;
minAngle = 0;
maxAngle = 2*pi;

% Pre-allocate variables used in loop (good for performance)
theta_1 = linspace(minAngle,maxAngle,nSamples);
theta_2 = linspace(minAngle,maxAngle,nSamples);
x = zeros(nSamples^2, 1);
y = zeros(nSamples^2, 1);

% Calculate EndEffector locations at sample intervals
for i = 1:nSamples
    for j = 1:nSamples
        
        % Calculate EndEffector transform and store x/y in linear index
        theta = [theta_1(i) theta_2(j)];
        index = (i-1) * nSamples + j;
        [x(index),  y(index)] = endEffector_RR(theta, linkLengths);
        
    end
end

% Plot all locations
fig = plot(x, y, 'o');
title('EndEffector workspace');
xlabel('x [m]');
ylabel('y [m]');
axis equal;

% Auto-limit axes
axis tight

end

% Hint: subfunctions can only be called from within the primary function
% in this file.
function [x, y] = endEffector_RR(theta, linkLengths)

    s1 = sin(theta(1));
    c1 = cos(theta(1));
    s12 = sin(theta(1) + theta(2));
    c12 = cos(theta(1) + theta(2));

    l1 = linkLengths(1);
    l2 = linkLengths(2);

    x = l1*c1 + l2*c12;
    y = l1*s1 + l2*s12;

end
