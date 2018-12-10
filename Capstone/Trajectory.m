% Manages Basic Information about an N-Dimensional Trajectory Parameterized
% by Path-Length, through Waypoints stored as a Matrix with Row-Vectors of
% Waypoints.
classdef Trajectory < handle
    properties
        points; % Matrix of Row Vectors of Waypoints
        dim; % Dimensionality of Trajectory
        numPts; % Number of Waypoints in Trajectory
        normals; % Matrix of Row Vectors of Normal Orientation of Trajectory at Each Point
        dist; % Distance (Path-Length) Traversed along Trajectory at Each Point
    end % Properties
    
    methods
        function obj = Trajectory(wps)
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
        end
        
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
        end
        
        % Plots the Points of the Trajectory and the Normals Along It
        function plot_pts(obj)
            hold on
                % Lines between Points:
                plot3(obj.points(:,1), obj.points(:,2), obj.points(:,3), 'b');
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
                
                % Ensure Axes are Scaled Equally (bounding cube):
                h = get(gca,'DataAspectRatio');
                if h(3)==1
                      set(gca,'DataAspectRatio',[1 1 1/max(h(1:2))])
                else
                      set(gca,'DataAspectRatio',[1 1 h(3)])
                end

        end % #plot_pts
    end % Methods
end % Class Trajectory