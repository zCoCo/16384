% Manages Basic Information about an N-Dimensional Trajectory Parameterized
% by Path-Length, through Waypoints stored as a Matrix with Row-Vectors of
% Waypoints.
classdef (Abstract) Trajectory < handle
    properties
        points; % Matrix of Row Vectors of Waypoints
        dim; % Dimensionality of Trajectory
        numPts; % Number of Waypoints in Trajectory
        normals; % Matrix of Row Vectors of Normal Orientation of Trajectory at Each Point
        dist; % Distance (Path-Length) Traversed along Trajectory at Each Point
        duration; % Total Time it Takes to Execute the Trajectory
        
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
    end % Properties
    
    methods
        % Initializes Trajectory and Computes All Metadata.
        % 
        % NOTE: Since concavity can change the sign of normal vectors and
        % cause discontinuities, use fixed point pf to ensure all normals 
        % are pointing in the same direction, away from pf.
        function obj = Trajectory(wps, pf)
            obj.points = wps;
            obj.numPts = size(obj.points,1);
            obj.dim = size(obj.points,2);
            if(obj.numPts < obj.dim)
                error('Trajectory Must have at least %d Points', obj.dim);
            end
            
            % Compute Normal Orientation of Trajectory at Each Point:
            if obj.dim == 3
                for i = 2 : obj.numPts-1
                    A = obj.points(i+1,:) - obj.points(i,:);
                    B = obj.points(i-1,:) - obj.points(i,:);
                    obj.normals(i,:) = cross(A,B);
                    obj.normals(i,:) = obj.normals(i,:) / norm(obj.normals(i,:)); % Normalize
                    
                    % Ensure Normals are Oriented Continuously (pointing
                    % away from pf).
                    pm = obj.points(i,:) - obj.normals(i,:);
                    pp = obj.points(i,:) + obj.normals(i,:);
                    if(norm(pm-pf) > norm(pp-pf))
                        % Pick Direction of Normal which Brings o5 Closest to Origin:
                        obj.normals(i,:) = -obj.normals(i,:);
                    end
                end
                % Ensure Continuity with Endpoint Normals:
                obj.normals(1,:) = obj.normals(2,:);
                obj.normals(end+1,:) = obj.normals(end,:);
            elseif obj.dim == 2
                for i = 2 : obj.numPts
                    A = obj.points(i,:) - obj.points(i-1,:);
                    obj.normals(i,1) = -A(2);
                    obj.normals(i,2) = A(1);
                    obj.normals(i,:) = obj.normals(i,:) / norm(obj.normals(i,:)); % Normalize
                    
                    % Ensure Normals are Oriented Continuously (pointing
                    % away from pf).
                    pm = obj.points(i,:) - obj.normals(i,:);
                    pp = obj.points(i,:) + obj.normals(i,:);
                    if(norm(pm-pf) > norm(pp-pf))
                        % Pick Direction of Normal which Brings o5 Closest to Origin:
                        obj.normals(i,:) = -obj.normals(i,:);
                    end
                end
                % Ensure Continuity with Endpoint Normals:
                obj.normals(1,:) = obj.normals(2,:);
            else
                % Default for One Dimensional Paths:
                obj.normals = -ones(obj.numPts, 1);
            end
            
            % Compute Cumulative Path Length Traversed at Each Point:
            obj.dist(1) = 0;
            for i = 2 : obj.numPts
                Ds = obj.points(i,:) - obj.points(i-1,:);
                obj.dist(i) = obj.dist(i-1) + norm(Ds);
            end
        end % ctor
        
        % Precomputes and Stores All Trajectory Information using the Given
        % dt.
        function precompute(obj, dt)
            obj.data.timestep = dt;
            Tf = obj.duration;
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
                obj.data.vs(i) = obj.v_t(t);
                obj.data.as(i) = obj.a_t(t);
                obj.data.js(i) = obj.j_t(t);
                
                i = i+1;
                t = t+dt;
            end
            % Add final point at T if dt doesn't go cleanly into Tf.
            if i == N
                obj.data.ts(i) = Tf;
                obj.data.xs(i) = obj.s_t(Tf);
                obj.data.vs(i) = obj.v_t(Tf);
                obj.data.as(i) = obj.a_t(Tf);
                obj.data.js(i) = obj.j_t(Tf);
            end
            
            obj.data.precomputed = true;
        end % #precompute
        
        % Returns the Interpolated Point at the Given Distance along the
        % Trajectory Path
        function p = point(obj, dist)
            if(dist < 0)
                p = obj.points(1,:);
            elseif(dist > obj.dist(end))
                p = obj.points(end,:);
            else
                p = zeros(1,size(obj.points,2)); % Preallocate for speed
                for i = 1 : size(obj.points,2)
                    p(i) = interp1(obj.dist, obj.points(:,i), dist, 'spline');
                end
            end
        end % #point
        
        % Returns the Interpolated Point at the Given Time into the
        % Trajectory Execution
        function p = point_t(obj,t)
            p = point(obj.s_t(t));
        end % #point_t
        
        % Returns the Interpolated Trajectory Normal at the Given Distance
        % along the Trajectory Path
        function n = normal(obj, dist)
            if(dist < 0)
                n = obj.normals(1,:);
            elseif(dist > obj.dist(end))
                n = obj.normals(end,:);
            else
                n = zeros(1,size(obj.normals,2)); % Preallocate for speed
                for i = 1 : size(obj.normals,2)
                    n(i) = interp1(obj.dist, obj.normals(:,i), dist, 'spline');
                end
            end
        end % #normal
        
        % Returns the Interpolated Trajectory Normal at the Given Time into
        % the Trajectory Execution
        function n = normal_t(obj,t)
            n = normal(obj.s_t(t));
        end % #normal_t
        
        % Returns the Interpolated Trajectory Tangent at the Given Distance
        % along the Trajectory Path
        function t = tangent(obj, dist, res)
            % Effective Sample Resolution (ideal number of points along
            % trajectory):
            if nargin < 3
                res = obj.dist(end) / max(1000, obj.numPts);
            end
            
            % Use interpolated sampling in case set of waypoints is
            % sparse and to ensure continuity with path:
            if dist - res*obj.dist(end) > 0
                t = obj.point(dist) - obj.point(dist - res);
            else
                t = obj.point(dist + res) - obj.point(dist);
            end
            t = t / norm(t); % Direction of Velocity Vector
        end % #tangent
        
        % Returns the Interpolated Trajectory Velocity Vector at the 
        % Given Distance along the Trajectory Path with sampling resolution
        % res (optional)
        function v = velocity_s(obj, s, res)
            if(s < 0)
                v = zeros(size(obj.points(1,:)));
            elseif(s > obj.dist(end))
                v = zeros(size(obj.points(1,:)));
            else
                v = obj.v_s(s) * obj.tangent(s,res); % Magnitude * tangent vector
            end
        end % #velocity_s
        
        % Returns the Interpolated Trajectory Velocity Vector at the Given 
        % Time into the Trajectory Execution
        function v = velocity_t(obj, t, res)
            if nargin > 2
                v = obj.velocity_s(obj.s_t(t),res);
            else
                v = obj.velocity_s(obj.s_t(t));
            end
        end % #velocity_t
        
        % Computes the Roll, Pitch, and Yaw Angles Required to Achieve the
        % Orientation Required at a Distance, s, along the Path. Returns as
        % a 3x1 row vector of the angles in the order: roll, pitch, yaw.
        %
        % NOTE: naxis and taxis specify which frame axes get rotated into
        % the normal and tangent vectors. Eg. if x is supposed to point
        % along the normal and y is supposed to point along the tangent,
        % naxis = 1, taxis = 2.
        function rpy = RPY_s(obj, s, naxis, taxis)
            if obj.dim ~= 3
                error('#RPY works in SE(3) (3-Dimensional Space)');
            end
            axes = [1,2,3];
            % Check Inputs:
            if prod(axes~=naxis) || prod(axes~=taxis)
                error('naxis and taxis must be 1, 2, or 3');
            end
            if naxis == taxis
                error('naxis and taxis must be different axes');
            end
                
            % Construct all Three Vectors:
            n = obj.normal(s);
            t = obj.tangent(s);
            a = cross(n,t);
            
            % Figure out which axis is undeclared
            aaxis = axes(axes~=naxis & axes~=taxis);
            
            % Construct Rotation Matrix:
            R(:,naxis) = n;
            R(:,taxis) = t;
            R(:,aaxis) = a;
            
            if(R(1,1) == 0 || R(2,1) == 0)
                yaw = atan2(R(1,2), R(2,2));
                pitch = pi/2;
                roll = 0;
            else
                yaw = atan2(R(3,2), R(3,3));
                pitch = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
                roll = atan2(R(2,1), R(1,1));
            end
            
            rpy = [roll, pitch, yaw];
        end % #RPY_s
        % Shorthand for RPY(s_t(t)):
        function rpy = RPY_t(obj,t)
            rpy = RPY_s(obj.s_t(t));
        end % #RPY_t
        
        % Plots the Points of the Trajectory and the Normals Along It
        function plot_pts(obj)
            hold on
                % Lines between Points:
                %plot3(obj.points(:,1), obj.points(:,2), obj.points(:,3), 'b');
                % Each Point:
                scatter3(obj.points(:,1), obj.points(:,2), obj.points(:,3));
                
                % Find Size of Path (Diagonal across Bounding Box):
                xmin = min(obj.points(:,1)); xmax = max(obj.points(:,1));
                ymin = min(obj.points(:,2)); ymax = max(obj.points(:,2));
                zmin = min(obj.points(:,3)); zmax = max(obj.points(:,3));
                sz = norm([xmax-xmin, ymax-ymin, zmax-zmin]);
                
                L = sz / 20; % Size of Each Normal Vector
                
                % Plot Normals
                for i = floor(linspace(1,obj.numPts, 50))
                    pts = obj.points(i,:);
                    pts(end+1,:) = pts(1,:) + L*obj.normals(i,:);
                    plot3(pts(:,1), pts(:,2), pts(:,3), 'g');
                end
                
                % Plot Tangents
                for i = floor(linspace(1,obj.numPts, 50))
                    pts = obj.points(i,:);
                    pts(end+1,:) = pts(1,:) + L*obj.tangent(obj.dist(i));
                    plot3(pts(:,1), pts(:,2), pts(:,3), 'r');
                end
                
            hold off
                
            % Ensure Axes are Scaled Equally (bounding cube):
            h = get(gca,'DataAspectRatio');
            if h(3)==1
                  set(gca,'DataAspectRatio',[1 1 1/max(h(1:2))])
            else
                  set(gca,'DataAspectRatio',[1 1 h(3)])
            end

        end % #plot_pts
        
        % Plots the Time-Parameterized Curves of the Trajectory
        function plot_t(obj)
            hold on
                plot(obj.data.ts, obj.data.xs, 'r');
                plot(obj.data.ts, obj.data.vs, 'g');
                plot(obj.data.ts, obj.data.as, 'k');
            hold off
            xlabel('Time [s]');
            legend('Position', 'Velocity', 'Acceleration');
        end % #plot_t
        
        % Plots the Position-Parameterized Curves of the Trajectory
        function plot_s(obj)
            hold on
                plot(obj.data.xs, obj.data.ts, 'r');
                plot(obj.data.xs, obj.data.vs, 'g');
                plot(obj.data.xs, obj.data.as, 'k');
            hold off
            xlabel('Path Position [units]');
            legend('Time Elapsed', 'Velocity', 'Acceleration');
        end % #plot_s
    end % methods
    
    methods (Abstract)
        % Gives the Time into the Trajectory Execution that Corresponds
        % with the Given Position along the Path
        t = t_s(obj,s)
        % Gives the Path Velocity at a Given Position along the Path
        v = v_s(obj,s)
        % Gives the Path Acceleration at a Given Position along the Path
        a = a_s(obj,s)
        % Gives the Path Jerk at a Given Position along the Path
        j = j_s(obj,s)
        
        % Gives the Position at a Given Time into the Trajectory Execution
        s = s_t(obj,t)
        % Alternative Function Name for Naming Consistency:
        x = x_t(obj,t)
        % Gives the Velocity at a Given Time into the Trajectory Execution
        v = v_t(obj, t)
        % Gives the Acceleration at a Given Time into the Trajectory Execution
        a = a_t(obj, t)
        % Gives the Jerk at a Given Time into the Trajectory Execution
        j = j_t(obj, t)
    end % abstract methods
end % Class Trajectory