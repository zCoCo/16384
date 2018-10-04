function [ ] = workspace_analysis_PR(l1)
% workspace_analysis_RR
%
%   This function visualizes the workspace for a RR configuration.
%   Use this as a basis for developing your RPR analysis.

%%
% Setup parameters
nSamples = 50;
minStroke = 0;
maxStroke = 2; % df
minAngle = 0;
maxAngle = 2*pi;

% Pre-allocate variables used in loop (good for performance)
d_1 = linspace(minStroke,maxStroke,nSamples);
theta_1 = linspace(minAngle,maxAngle,nSamples);
x = zeros(nSamples^2, 1);
y = zeros(nSamples^2, 1);

% Calculate EndEffector locations at sample intervals
for i = 1:nSamples
    for j = 1:nSamples
        
        % Calculate EndEffector transform and store x/y in linear index
        config = [d_1(i) theta_1(j)];
        index = (i-1) * nSamples + j;
        [x(index),  y(index)] = endEffector_PR(config, l1);
        
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
function [x, y] = endEffector_PR(config, l1)

    s1 = sin(config(2));
    c1 = cos(config(2));

    x = l1*c1;
    y = config(1) + l1*s1;

end
