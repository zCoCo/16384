classdef RobotVisualizer2D < handle
    %RobotVisualizer2D draws the coordinate frames of each section in a kinematic
    %chain. The axes are color-coded RGB for XY.
    %
    %  visualizer = RobotVisualizer2D(axisLength[m]) opens a new
    %  figure for drawing coordinate frames. Each coordinate frame's 
    %  axes  are of specified length. 
    %
    %  Notes: 
    %
    %     The figure gets closed automatically when this variable 
    %     is deleted.
    %  
    %     If no axisLength argument is specified, the constructor 
    %     assumes a common default.
    %
    %  The frames can be updated using the 'setFrames' function, which
    %  accepts a 3x3xN matrix. each 3x3 matrix corresponds to a homogeneous 
    %  transform between the base frame and the element. N is the number of
    %  elements, e.g., the total number of joints and links.
    %
    %  x - red
    %  y - green
    %
    %  Example:
    %     % Initialize
    %     visualizer = RobotVisualizer2D();
    %     frames = eye(3); % 3x3x1
    %     visualizer.setFrames(frames);
    
    properties (Access = private)
        figHandle = [];
        x
        y
        link
        numFrames
        axisLength
    end
    
    methods(Access = public)
        
        function this = RobotVisualizer2D(numFrames, axisLength, createNewFig)
            
            % Default axisLength if not supplied
            if nargin < 2
               axisLength = 0.1; % [m] 
            end
            this.axisLength = axisLength;
            
            % If numFrames is supplied, initialize immediately
            if nargin > 0
                if nargin < 3 
                    this.initializeFigure(numFrames, true);
                else
                    this.initializeFigure(numFrames, createNewFig);
                end
                
            end
            
        end
        
        function setFrames(this, frames)
            
            % Initialize once
            if isempty(this.figHandle)
                n = size(frames, 3);
                this.initializeFigure(n, true);
            end
            
            % Draw gets called often and continuously updates the frames
            if this.numFrames == 1 
                if ~isequal(size(frames), [3 3])
                  error('expected input: 3x3');  
                end
            else
                if ~isequal(size(frames), [3 3 this.numFrames])
                    error(['expected input: 3x3x' num2str(this.numFrames)]);
                end
            end
            
            axes_base = [eye(2) * this.axisLength; ones(1,2)];
            orig_base = [zeros(2); ones(1,2)];
            for i = 1:this.numFrames
                
                % Extract transforms
                T = frames(:,:,i);
                if i > 1
                    T_previous = frames(:,:,i-1);
                else
                    T_previous = eye(3);
                end
                
                % Transform principle axes to the position / orientation of
                % each module.
                axes_T = T * axes_base;
                orig_T = T * orig_base;
                
                % Update existing line handles
                set( this.x(i), ...
                    'XData', [orig_T(1,1) axes_T(1,1)], ...
                    'YData', [orig_T(2,1) axes_T(2,1)]);
                set( this.y(i), ...
                    'XData', [orig_T(1,2) axes_T(1,2)], ...
                    'YData', [orig_T(2,2) axes_T(2,2)] );
                
                % Draw links between elements
                start = T_previous * [0 0 1]';
                finish = T * [0 0 1]';
                set( this.link(i), ...
                    'XData', [start(1) finish(1)], ...
                    'YData', [start(2) finish(2)] );
            end
            
        end
        
    end
    
    methods (Access = private)
        
        function [] = initializeFigure(this, numFrames, createNewFig)
            
            if createNewFig
                this.figHandle = figure();
            else
                this.figHandle = gcf();
            end
            
            hold on;
            this.numFrames = numFrames;
            
            % Initialize all axes to identity. Iterate backwards so that
            % we don't need to pre-initialize arrays
            range = [0 this.axisLength];
            for i = numFrames:-1:1
                this.x(i) = line( range, [0 0], 'Color', 'r', 'LineWidth', 2);
                this.y(i) = line( [0 0], range, 'Color', 'g', 'LineWidth', 2);
                this.link(i) = line( [0 0], [0 0], 'Color', 'k', 'LineWidth', 2,  'LineStyle', ':');
            end
            
            % labels
            legend('x', 'y', 'location', 'southoutside');
            title('Frame Display 2D');
            xlabel('x');
            ylabel('y');
            
            
            % Draw small coordinate frame in the center
            line( range, [0 0], 'Color', 'k', 'LineWidth', 2);
            line( [0 0], range, 'Color', 'k', 'LineWidth', 2);
            grid minor;
            box on;
            axis equal;
            hold off;
            view(2);
            
            % Limit the range to +/- 1 meter by default
            d = 1;
            xlim([-d d]); ylim([-d d]);
            
        end
        
        function delete(this)
            % close figure if it is still open
            if any(findall(0,'Type','Figure') == this.figHandle)
                close(this.figHandle);
            end
        end
    end
    
end

