% Manages Basic Information about an N-Dimensional Trajectory Parameterized
% by Path-Length, Loaded from Data in a CSV File with Row-Vectors of
% Waypoints.
classdef Trajectory < handle
    properties
        points; % Matrix of Row Vectors of Waypoints
        dim; % Dimensionality of Trajectory
        numPts; % Number of Waypoints in Trajectory
        normals; % Normal Orientation of Trajectory at Each Point
        distance; % Distance (Path-Length) Traversed along Trajectory at Each Point
    end % Properties
    
    methods
        function obj = Trajectory(csv_filename)
            obj.points = csvread(csv_filename);
            obj.numPts = size(obj.points,1);
            obj.dim = size(obj.points,2);
            if(obj.numPts < obj.dim)
                error('Trajectory Must have at least %d Points', obj.dim);
            end
            
            % Compute Normal Orientation of Trajectory at Each Point:
            for i = 2 : obj.numPts-1
                A = obj.points(i+1,:) - obj.points(i,:);
                B = obj.points(i-1,:) - obj.points(i,:);
                obj.normals(i,:) = cross(A,B);
            end
            % Ensure Continuity with Endpoint Normals:
            obj.normals(1,:) = obj.normals(2,:);
            obj.normals(end,:) = obj.normals(end-1,:);
            
            % Compute Path Length Traversed at Each Point:
            obj.distance(1) = 0;
            for i = 2 : obj.numPts
                Ds = obj.points(i,:) - obj.points(i-1,:);
                obj.distance(i) = obj.distance(i-1) + norm(Ds);
            end
        end % ctor
        
        % Returns the Interpolated Point at the Given Distance along the
        % Trajectory
        function p = point(obj, dist)
            if(dist < 0)
                p = obj.points(1,:);
            elseif(dist > obj.distance(end))
                p = obj.points(end,:);
            else
                for i = 1 : size(obj.points,2)
                    p(i) = interp1(obj.distance, obj.points(:,i), dist, 'spline');
                end
            end
        end
        
        % Returns the Interpolated Trajectory Normal at the Given Distance
        % along the Trajectory
        function n = normal(obj, dist)
            if(dist < 0)
                n = obj.normals(1,:);
            elseif(dist > obj.distance(end))
                n = obj.normals(end,:);
            else
                for i = 1 : size(obj.normals,2)
                    n(i) = interp1(obj.distance, obj.normals(:,i), dist, 'spline');
                end
            end
        end
    end % Methods
end % Class Trajectory