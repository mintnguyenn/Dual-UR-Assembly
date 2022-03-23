classdef UFOFleet < handle
    %UFO A way of creating a group of UFOs
    %   
    
    properties (Constant)                
        %> flightVolume in meters
        flightVolume = [10,10,10]; 
        
        %> Radius (approx) of the UFO, which could be computed based upon
        %the points
        shipRadius = 0.8;
    end
    
    properties
        %> Number of ParrotQuadrotor
        count = 2;
        
        %> A cell structure of \c count models
        model;
               
        %> Dimensions of the workspace in regard to the flightVolume
        workspaceDimensions;
        
        %> Max health points 
        maxHealth = 20;
        
        %> Health points remaining
        healthRemaining;
        
        %> Ply file data about the model
        faceData = [];
        vertexData = [];
        plyData = [];
    end
    
    methods
        %% ...structors
        function self = UFOFleet(count)
            if 0 < nargin
                self.count = count;
            end
            
            self.workspaceDimensions = [-self.flightVolume(1)/2, self.flightVolume(1)/2 ...
                                       ,-self.flightVolume(2)/2, self.flightVolume(2)/2 ...
                                       ,0, self.flightVolume(3)];

            self.healthRemaining = zeros(1,self.count);
            % Create the required number of UFOs
            for i = 1:self.count
                self.model{i} = self.GetModel(['UFO',num2str(i)]);
                % Random spawn
                self.model{i}.base = se3(se2((2 * rand()-1) * self.flightVolume(1)/2 ...
                                           , (2 * rand()-1) * self.flightVolume(2)/2 ...
                                           , (2 * rand()-1) * 2 * pi));
                 % Random height (above 2m)
                self.model{i}.base = self.model{i}.base * transl(0,0,2 + rand * (self.flightVolume(3)-2));
                 % Plot 3D model
                 self.model{i}.animate(0);
%                 plot3d(self.model{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0);

                self.healthRemaining(i) = self.maxHealth;
                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
                
            end

            axis equal
            camlight;
        end
        
        function delete(self)
%             cla;
        end       
        
        %% PlotSingleRandomStep
        % Move each of the UFOs forward and rotate some rotate value around
        % the z axis
        function PlotSingleRandomStep(self)
            for index = 1:self.count
                % If either 0 or -1 health left then don't plot a move
                if self.healthRemaining(index) < 1
                    continue;
                end
                % Move Forward
                self.model{index}.base = self.model{index}.base * se3(se2(0.2, 0, 0));
                animate(self.model{index},0);
                % Move Up (or down)
                self.model{index}.base = self.model{index}.base *  transl(0,0, (rand - 0.5) * 0.2);
                animate(self.model{index},0);
                
                % Turn randomly
                self.model{index}.base(1:3,1:3) = self.model{index}.base(1:3,1:3) *  rotz((rand-0.5) * 30 * pi/180);
                animate(self.model{index},0);                

                % If outside workspace rotate back around
                if self.model{index}.base(1,4) < self.workspaceDimensions(1) ...
                || self.workspaceDimensions(2) < self.model{index}.base(1,4) ...
                || self.model{index}.base(2,4) < self.workspaceDimensions(3) ...
                || self.workspaceDimensions(4) < self.model{index}.base(2,4) ...
                || self.model{index}.base(3,4) < self.workspaceDimensions(5) ...
                || self.workspaceDimensions(6) < self.model{index}.base(3,4)
                    self.model{index}.base = self.model{index}.base * se3(se2(-0.2, 0, 0)) * se3(se2(0, 0, pi));
                end
                
                % Move up since too low (at least 1 m above bottom)
                if self.model{index}.base(3,4) < self.workspaceDimensions(5) + 1
                    self.model{index}.base = self.model{index}.base * transl(0,0,0.2);
                end

                % Move down since too high
                if self.workspaceDimensions(6) < self.model{index}.base(3,4)
                    self.model{index}.base = self.model{index}.base * transl(0,0,-0.2);
                end
            end
            % Do the drawing once for each interation for speed
            drawnow();
        end    
        
        %% RemoveDead
        function RemoveDead(self)
            for index = 1:self.count
                if self.healthRemaining(index) == 0
                    handles = findobj('Tag', self.model{index}.name);
                    h = get(handles,'UserData');
                    
                    % Make big and red (explode)
                    h.link(1).Children.XData = h.link(1).Children.XData * 1.5;
                    h.link(1).Children.YData = h.link(1).Children.YData * 1.5;
                    h.link(1).Children.ZData = h.link(1).Children.ZData * 1.5; 
                    h.link(1).Children.FaceColor = [1,0,0];                    
                    drawnow();

                    self.model{index}.base = transl(0,0,0);
                    % Then make UFO invisible
                    h.link(1).Children.FaceColor = 'none';                    
                    h.link(1).Children.FaceVertexCData = [];
                    h.link(1).Children.CData = [];                    
                    h.link(1).Children.XData = h.link(1).Children.XData * eps;
                    h.link(1).Children.YData = h.link(1).Children.YData * eps;
                    h.link(1).Children.ZData = h.link(1).Children.ZData * eps;                    
                    % Don't try and remove again
                    self.healthRemaining(index) = -1;
                    animate(self.model{index},0);
                end
            end
        end
        
        %% TestPlotManyStep
        % Go through and plot many random walk steps
        function TestPlotManyStep(self,numSteps,delay)
            if nargin < 3
                delay = 0;
                if nargin < 2
                    numSteps = 200;
                end
            end
            for i = 1:numSteps
                self.PlotSingleRandomStep();
                pause(delay);
            end
        end
        
        %% SetHit
        function SetHit(self,ufoHitIndex)
            % If not hits then return
            if isempty(ufoHitIndex)
                return;
            end
            % Otherwise go through the hits and update colors and/or remove
            for hitIndex = ufoHitIndex
                % If any health left then change color and remove health points
                if self.healthRemaining(hitIndex) < 1
                    continue;
                end
                self.healthRemaining(hitIndex) = self.healthRemaining(hitIndex) - 1;

                % Shift to red
                handles = findobj('Tag', self.model{hitIndex}.name);
                h = get(handles,'UserData');
                h.link(1).Children.FaceVertexCData(:,2) = h.link(1).Children.FaceVertexCData(:,2) * 0.2;
                h.link(1).Children.FaceVertexCData(:,3) = h.link(1).Children.FaceVertexCData(:,3) * 0.2;
            end
            self.RemoveDead();
        end

        %% GetModel
        function model = GetModel(self,name)
            if nargin < 1
                name = 'UFO';
            end
            if isempty(self.faceData) || isempty(self.vertexData) || isempty(self.plyData)
                [self.faceData,self.vertexData,self.plyData] = plyread('Haneubu+uf.ply','tri');
            end
            L1 = Link('alpha',0,'a',1,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {self.faceData,[]};
            model.points = {self.vertexData,[]};
                                   
            plot3d(model,0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0);
            handles = findobj('Tag', model.name);
            h = get(handles,'UserData');

%             h.link(1).Children.FaceVertexCData = [data.face.red,data.face.green,data.face.blue];
%             h.link(1).Children.CData = [data.face.red,data.face.green,data.face.blue]/255;
            h.link(1).Children.FaceVertexCData = [self.plyData.vertex.red ...
                                                 ,self.plyData.vertex.green ...
                                                 ,self.plyData.vertex.blue]/255;
%             h.link(1).Children.FaceVertexCData = [data.vertex.red,data.vertex.green,data.vertex.blue]'/255;
            h.link(1).Children.FaceColor = 'interp';
        end
    end    
end

