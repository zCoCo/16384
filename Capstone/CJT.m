% Creates a Time and Distance Parameterized Constant Jerk Trajectory along
% a Given Path, Loaded from Data in a CSV File with Row-Vectors of
% Waypoints.
classdef CJT < Trajectory
    properties(SetAccess = private, GetAccess = public)
        jmax; %     Maximum Allowable Jerk
        amax; %     Maximum Allowable Acceleration
        vmax; %     Maximum Allowable Velocity
        
        % Precomputed Trajectory Path Data:
        data = struct( ...
            'precomputed', false, ... % Whether Data has been Precomputed
            'timestep', 0, ... %          Timestep for Precomputed Data 
            'ts', [], ... %             Timestamps for Each Data Point
            'xs', [], ... %             Positions along Path at Each Timestamp
            'vs', [], ... %             Velocities along Path at Each Timestamp
            'as', [], ... %             Accelerations along Path at Each Timestamp
            'js', [] ... %              Jerk along Path at Each Timestamp
        );
    
        % Parameters Defining Trajectory Curve:
        %{
        Relevant Critical Points:
        1: End of Acceleraing Concave Velocity Region
        2: End of Accelerating Linear Velocity Region
        3: End of Accelerating Convex Velocity Region
        4: End of Constant Velocity Region
        5: End of Decelerating Convex Velocity Region
        6: End of Decelerating Linear Velocity Region
        7: End of Decelerating Concave Velocity Region (and end of path)
        
        N.B. If a region doesn't exist (eg. between 1 and 2), their
        critical times and positions will be the same so that any
        comparisions against a given time or position will never land
        inside them.
        %}
        params = struct( ...
            'apeak', 0, ... %   Peak Acceleration in Path
            'vpeak', 0, ... %   Peak Velocity in Path
            'tcrit', [], ... %  Times at each critical point
            'xcrit', [] ... %   Path positions at each critical point
        );
            
    end
    methods
        % Computes all Parameters for a CJT Passing through the Waypoints
        % in the Specified CSV File with the given maximum allowable 
        % acceleration, jerk, and velocity. Optionally precomputes data
        % for x, v, a, j across the entire path if a timestep, dt, is
        % given.
        function obj = CJT(csv_filename, jm, am, vm, dt) 
            obj = obj@Trajectory(csv_filename); % Call Superclass ctor
            
            obj.jmax = jm;
            obj.amax = am;
            obj.vmax = vm;
            
            obj.generateTrajectoryParameters();
            
            if nargin > 4
                obj.precompute(dt);
            end
        end % ctor
        
        % Precomputes and Stores All Trajectory Information using the Given
        % dt.
        function precompute(obj, dt)
            obj.data.timestep = dt;
            Tf = obj.params.tcrit(end);
            % If Tf is not clean multiple of dt, add one extra data point
            % at end, exactly at Tf.
            N = Tf/dt;
            N = floor(N) + ceil(N - floor(N));
            
            % Pre-allocate Data:
            obj.data.ts = zeros(1,N);
            obj.data.xs = zeros(1,N);
            obj.data.vs = zeros(1,N);
            obj.data.as = zeros(1,N);
            obj.data.js = zeros(1,N);
            
            t = 0;
            i = 1;
            while(t < Tf)
                obj.data.ts(i) = t;
                obj.data.xs(i) = obj.s_t(t);
                obj.data.as(i) = obj.v_t(t);
                obj.data.vs(i) = obj.a_t(t);
                obj.data.js(i) = obj.j_t(t);
                
                i = i+1;
                t = t+dt;
            end
            % Add final point at T if dt doesn't go cleanly into Tf.
            if i == N
                obj.data.ts(i) = Tf;
                obj.data.xs(i) = s_t(Tf);
                obj.data.as(i) = v_t(Tf);
                obj.data.vs(i) = a_t(Tf);
                obj.data.js(i) = j_t(Tf);
            end
            
            obj.data.precomputed = true;
        end % #precompute
        
        % Determine All Key Parameters for the Trajectory:
        function generateTrajectoryParameters(obj)
            vp = obj.vmax; % Peak Attainable Velocity
            ap = obj.amax; % Peak Attainable Acceleration
            
            tr = obj.amax / obj.jmax; % Ramp Time of Acceleration Trapezoid
            tc = obj.vmax / obj.amax + tr; % Time at Half-Way Mark (peak velocity)
            thold = 0; % Time Max Velocity Must be Held at Middle of Path
            
            if(obj.amax^2 / obj.jmax > obj.vmax)
                % Not enough time to smoothly go up to amax and back down 
                % before hitting vmax
                ap = sqrt(obj.jmax * obj.vmax); % Recompute Max Attainable Accel.
                tr = ap / obj.jmax;
                tc = 2 * tr;
            end
            
            % Determine position reached when max velocity is attained:
            if tr < tc/2
                % There is a linear portion in the middle of the velocity
                % curve
                v1 = ap*tr/2; % Velcity at end of concave region
                v2 = v1 + ap * (tc-2*tr);
            else
                v1 = ap*tr/2; v2 = v1;
            end
            % Change in position during convex region of velocity profile:
            s2c = tr * (v2 + ap*tr/2 + 7 * obj.jmax*tr^2 / 12 - 9*obj.jmax*tc*tr/12);
            sc = ap*tr^2 / 6 + (v1+v2)*(tc-2*tr)/2 + s2c; % Position at max v
            
            if sc > obj.dist(end) / 2
                % Not enough distance in path to reach vmax; readjust peak
                % velocity accordingly.
                % First, assume amax will not attainable during curve:
                vp = (2 * sqrt(obj.jmax) * obj.dist(end))^(2/3);
                ap = sqrt(obj.jmax * vp);
                if ap > obj.amax
                    % But, if amax would be surpassed, recompute vp
                    % accordingly:
                    jm = obj.jmax;
                    vp = (am*(am + ((am^3 + 16*sf*jm^2)/am)^(1/2)))/(4*jm);
                    ap = obj.amax;
                    tr = ap / jm;
                    tc = vp / ap + tr;
                else
                    tr = ap / obj.jmax;
                    tc = 2*tr;
                end
                % Recompute Trajectory Landmarks:
                v1 = ap*tr/2;
                v2 = v1 + ap * (tc-2*tr);
                s2c = tr * (v2 + ap*tr/2 + 7 * obj.jmax*tr^2 / 12 - 9*obj.jmax*tc*tr/12);
                sc = ap*tr^2 / 6 + (v1+v2)*(tc-2*tr)/2 + s2c; % Position at max v
            else
                % Vmax must held for a while at the middle of the path to
                % be able to reach the end of the path
                thold = (obj.dist/2 - sc) / obj.vmax;
            end
            
            obj.params.apeak = ap;
            obj.params.vpeak = vp;
            obj.params.tcrit = [...
                tr, ... % End of Acceleraing Concave Velocity Region
                tc-tr, ... % End of Accelerating Linear Velocity Region
                tc, ... % End of Accelerating Convex Velocity Region
                tc + 2*thold, ... % End of Constant Velocity Region
                tc + 2*thold + tr, ... % End of Decelerating Convex Velocity Region
                2*(tc + thold) - tr, ... % End of Decelerating Linear Velocity Region
                2*(tc + thold) ... % End of Decelerating Concave Velocity Region
            ];
            sdecel = (obj.dist(end) - sc); % Path Position where Deceleration Begins
            obj.params.xcrit = [...
                ap*tr^2 / 6, ...
                sc - s2c,...
                sc, ...
                sdecel, ...
                sdecel + s2c, ...
                sdecel + sc - ap*tr^2 / 6, ...
                sdecel + sc;
            ];
        end % #generateTrajectoryParameters
        
        % Gives the Position at a Given Time into the Trajectory Execution
        function s = s_t(obj,t)
            % Useful shorthand for better readability:
            ap = obj.params.apeak;
            vp = obj.params.vpeak;
            tr = obj.params.tcrit(1);
            tc = obj.params.tcrit(3);
            
            % Determine Stage in Trajectory:
            if t < 0
                s = 0;
            elseif t < obj.params.tcrit(1)
                s = ap * t^3 / 6 / tr;
                
            elseif t < obj.params.tcrit(2)
                v1 = ap * tr /2;
                v2 = v1 + ap * (t - tr);
                s = obj.params.xcrit(1) + (v1+v2)*(t-tr)/2;
                
            elseif t < obj.params.tcrit(3)
                v2 = ap * tr /2 + ap * (tc - 2*tr);
                jm = obj.jmax;
                s2c = ((t - tc + tr)*(- 2*jm*t^2 - 5*jm*t*tc + 5*jm*t*tr + 6*ap*t + 7*jm*tc^2 - 14*jm*tc*tr - 6*ap*tc + 7*jm*tr^2 + 6*ap*tr + 12*v2))/12;
                s = obj.params.xcrit(2) + s2c;
                
            elseif t < obj.params.tcrit(4)
                s = obj.params.xcrit(3) + vp * (t - obj.params.tcrit(3));
                
            elseif t < obj.params.tcrit(5)
                v2 = ap * tr /2 + ap * (tc - 2*tr);
                jm = obj.jmax;
                sc = obj.params.xcrit(3);
                teff = obj.params.tcrit(5) - t + obj.params.tcrit(2);
                s2c = ((teff - tc + tr)*(7*jm*tc^2 - 5*jm*tc*teff - 14*jm*tc*tr - 6*ap*tc - 2*jm*teff^2 + 5*jm*teff*tr + 6*ap*teff + 7*jm*tr^2 + 6*ap*tr + 12*v2))/12;
                s = obj.params.xcrit(4) + (sc - s2c - obj.params.xcrit(2));
                
            elseif t < obj.params.tcrit(6)
                teff = obj.params.tcrit(6) - t + obj.params.tcrit(1);
                v1 = ap * tr /2;
                v2 = v1 + ap * (teff - tr);
                s = obj.params.xcrit(5) + obj.params.xcrit(2) - obj.params.xcrit(1) - (v1+v2)*(teff-tr)/2;
            elseif t < obj.params.tcrit(7)
                teff = obj.params.tcrit(7) - t;
                s = obj.params.xcrit(6) + obj.params.xcrit(1) - ap * teff^3 / 6 / tr;
            else
                s = obj.dist(end);
            end
        end % #s_t
        % Alternative Function Name for Naming Consistency:
        function x = x_t(obj,t); x = obj.s_t(t); end
        
        % Gives the Velocity at a Given Time into the Trajectory Execution
        function v = v_t(obj, t)
            % Useful shorthand for better readability:
            ap = obj.params.apeak;
            tr = obj.params.tcrit(1);
            tc = obj.params.tcrit(3);
            
            if t < 0
                v = 0;
            elseif t < obj.params.tcrit(1)
                v = ap * t^2 / tr / 2;
            elseif t < obj.params.tcrit(2)
                v = ap*tr/2 + ap * (t - tr);
            elseif t < obj.params.tcrit(3)
                jm = obj.jmax;
                v = ap*tr/2 + ap*(tc - 2*tr) + ap*(t-tc+tr) - jm * t^2 / 2 + jm * (tc-tr)^2 / 2 + jm*(tr-tc)*(t-tc+tr)/2;
            elseif t < obj.params.tcrit(4)
                v = obj.params.vpeak;
            elseif t < obj.params.tcrit(5)
                teff = obj.params.tcrit(5) - t + obj.params.tcrit(2);
                jm = obj.jmax;
                v = ap*tr/2 + ap*(tc - 2*tr) + ap*(teff-tc+tr) - jm * teff^2 / 2 + jm * (tc-tr)^2 / 2 + jm*(tr-tc)*(teff-tc+tr)/2;
            elseif t < obj.params.tcrit(6)
                teff = obj.params.tcrit(6) - t + obj.params.tcrit(1);
                v = ap*tr/2 + ap * (teff - tr);
            elseif t < obj.params.tcrit(7)
                teff = obj.params.tcrit(7) - t;
                v = ap * teff^2 / tr / 2;
            else
                v = 0;
            end
        end % #v_t
        
        % Gives the Acceleration at a Given Time into the Trajectory Execution
        function a = a_t(obj, t)
            % Useful shorthand for better readability:
            ap = obj.params.apeak;
            tr = obj.params.tcrit(1);
            
            if t < 0
                a = 0;
            elseif t < obj.params.tcrit(1)
                a = ap * t / tr;
            elseif t < obj.params.tcrit(2)
            	a = ap;
            elseif t < obj.params.tcrit(3)
                a = ap - ap * (t - obj.params.tcrit(2)) / tr;
            elseif t < obj.params.tcrit(4)
                a = 0;
            elseif t < obj.params.tcrit(5)
                a = - ap * (t - obj.params.tcrit(4)) / tr;
            elseif t < obj.params.tcrit(6)
                a = -ap;
            elseif t < obj.params.tcrit(7)
                a = -ap + ap * (t - obj.params.tcrit(6)) / tr;
            else
                a = 0;
            end
        end % #a_t
        
        % Gives the Jerk at a Given Time into the Trajectory Execution
        function j = j_t(obj, t)
            if t < 0
                j = 0;
            elseif t < obj.params.tcrit(1)
                j = obj.jmax;
            elseif t < obj.params.tcrit(2)
            	j = 0;
            elseif t < obj.params.tcrit(3)
                j = - obj.jmax;
            elseif t < obj.params.tcrit(4)
                j = 0;
            elseif t < obj.params.tcrit(5)
                j = - obj.jmax;
            elseif t < obj.params.tcrit(6)
                j = 0;
            elseif t < obj.params.tcrit(7)
                j = obj.jmax;
            else
                j = 0;
            end
        end % #j_t
    end
end % Class CJT