function [] = setup()
currentDir = fileparts(mfilename('fullpath'));

% Libraries
addpath(fullfile(currentDir , 'lib', 'hebi'));

% Exercises
addpath(fullfile(currentDir , 'ex_01'));

end
