
classdef CJT < Trajectory
    properties
        timestep; % Timestep for Motion 
        jmax; % Maximum Jerk
        amax; % Maximum Acceleration
        vmax; % Maximum Velocity
        
        % Trajectory Path Data:
        data = struct( ...
            'ts', [], ...
            'xs', [], ...
            'vs', [], ...
            'as', [], ...
            'js', [] ...
        );
            
    end
    methods
        function obj = CJT(dt, jm, am, vm) 
            obj.jmax = jm;
            obj.amax = am;
            obj.vmax = vm;
            obj.timestep = dt;
            
            obj.generateTrajectory();
        end % ctor
        
        function generateTrajectory(obj)
            dt = obj.timestep;
            s = 0; % Current Path Position
            v = 0; % Current Velocity
            a = 0; % Current Acceleration
            j = obj.jmax; % Current Jerk
            
            vp = obj.vmax; % Peak Attainable Velocity
            ap = obj.amax; % Peak Attainable Acceleration
            
            while se < obj.dist(end)/2
                % Multi-layer Midpoint Algorithm:
                if a + j*dt <= ap
                    a = a + j*dt/2;
                elseif a < ap
                    a = ap;
                end
                if v + a*dt <= vp
                    v = v + a*dt/2;
                elseif v < vp
                    v = vp;
                    ap = a; % Also, stop increasing acceleration
                end
                
                s = s + v*dt;
                
                if v < obj.vmax
                    v = v + a*dt/2;
                end
                if a < obj.amax
                    a = a + j*dt/2;
                end
            end
            
            
        end % #generateTrajectory
    end
end % Class CJT