function [] = setup()
currentDir = fileparts(mfilename('fullpath'));

% Libraries
addpath(fullfile(currentDir , 'lib', 'hebi'));

% Exercises
addpath(fullfile(currentDir , 'ex_01'));
addpath(fullfile(currentDir , 'ex_02'));
addpath(fullfile(currentDir , 'ex_03'));
addpath(fullfile(currentDir , 'ex_04'));

end
