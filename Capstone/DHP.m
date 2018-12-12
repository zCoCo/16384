% Class for Managing a Set of Denavit-Hartenberg Parameters for an
% Articulated Robot.
%
% NOTE: If a cell in the DHP matrix is actuated, leave its value at 0
% UNLESS an offset is required (ex. th1 = q1 + 90deg or d1 = d1 + 200), 
% then set the value to that offset. In #atConfig, configuration states 
% are simply added to the values in the matrix.
classdef DHP < handle
    properties(SetAccess = private, GetAccess = public)
        data = struct( ... % Packaged as struct so that it can be templated. (see #atConfig)
            'mat', [], ... %    - Matrix of Parameters (columns: a,alpha,d,theta)
            'as', [], ... %     - Vector of Constant (unactuated) 'a' Values
            'alphas', [], ...%  - Vector of Constant (unactuated) 'alpha' Values
            'ds', [], ... %     - Vector of Constant (unactuated) 'd' Values
            'thetas', [] ... %  - Vector of Constant (unactuated) 'theta' Values
        );
        actuatedJoints %        - Vector Indicating which Column in the DHP Matrix is Actuated (0 if not) for Each Row - should always be 3 or 4.
    end
    
    methods
        % Constructs a DHP Object from the Given Denavit-Hartenberg
        % Parameter Matrix with Columns: a, alpha, d, theta, and the Vector
        % of Actuated Joint Indices, aj.
        function dhp = DHP(mat, aj)
            dhp.actuatedJoints = aj;
            dhp.updateMatrix(mat);
        end % ctor
        
        % Updates the DHP Matrix and All Associated Values.
        function updateMatrix(dhp, mat)
            if size(mat, 2) ~= 4
                error('Invalid Denavit-Hartenberg Parameter Matrix: should have four columns');
            end
            
            dhp.data = DHP.matToData(mat);
        end % #updateMatrix
        
        
        % Returns the Complete DHP Data for the Robot in the Given Joint 
        % State Configuration.
        function cfgData = atConfig(dhp, cfg)
            if length(cfg) ~= size(dhp.data.mat, 1)
                error('Invalid Joint Configuration State. Vector should have as many cells as rows in DHP matrix, which is %d.', size(dhp, 1));
            end
            cm = dhp.data.mat;
            
            for i = 1:size(cm,1)
                if dhp.actuatedJoints(i) ~= 0
                    cm(i, dhp.actuatedJoints(i)) = cm(i, dhp.actuatedJoints(i)) + cfg(i);
                end
            end
            
            cfgData = DHP.matToData(cm);
        end % #atConfig
    end  % Public Instance Methods
    
    methods(Static)
        % Packages up the Given Matrix as a Data Struct. Ensures consistent
        % formatting everywhere. Using data as fields in a structure allows
        % for easily changing the way the values are computed later on and
        % allows for passing multiple data packages around.
        function data = matToData(mat) 
            data = struct();
            data.mat = mat;
            data.as = mat(:,1);
            data.alphas = mat(:,2);
            data.ds = mat(:,3);
            data.thetas = mat(:,4);
        end % #matToData
        
        % Returns the Intermediate Transform for the ith Row of the DHP
        % Matrix of the Given DHP Data Package.
        function frame = transform(data, i)
            % Shorthand to Ease Reading:
            ct = @(nn) cos(data.thetas(nn)); % Returns the cos of the DH theta for frame nn
            st = @(nn) sin(data.thetas(nn)); % Returns the sin of the DH theta for frame nn
            ca = @(nn) cos(data.alphas(nn)); % Returns the cos of the DH alpha for frame nn
            sa = @(nn) sin(data.alphas(nn)); % Returns the sin of the DH alpha for frame nn

            a = @(nn) data.as(nn); % Returns the DH a offset for frame nn
            d = @(nn) data.ds(nn); % Returns the DH d offset for frame nn

            % Compute frame for first joint (1st frame):
            frame = [...
                ct(i), -st(i)*ca(i), st(i)*sa(i), a(i)*ct(i);...
                st(i), ct(i)*ca(i), -ct(i)*sa(i), a(i)*st(i);...
                0, sa(i), ca(i), d(i);...
                0,0,0,1 ...
            ];
        end % #transform
    end % Static Methods
    
end % classdef DHP