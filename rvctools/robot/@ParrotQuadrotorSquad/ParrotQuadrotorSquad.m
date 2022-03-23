classdef ParrotQuadrotorSquad < handle
    %ParrotQuadrotor A way of creating a group of ParrotQuadrotors
    %   The uavs can be moved around randomly. It is then possible to query
    %   the current location (base) of the uav.
    
    properties (Constant)                
        %> flightVolume in meters
        flightVolume = [10,10,10]; 
    end
    
    properties
        %> Number of ParrotQuadrotor
        count = 2;
        
        %> A cell structure of \c count models
        uav;
               
        %> Dimensions of the workspace in regard to the flightVolume
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = ParrotQuadrotorSquad(count)
            if 0 < nargin
                self.count = count;
            end
            
            self.workspaceDimensions = [-self.flightVolume(1)/2, self.flightVolume(1)/2 ...
                                       ,-self.flightVolume(2)/2, self.flightVolume(2)/2 ...
                                       ,-self.flightVolume(3)/2, self.flightVolume(3)/2];

            % Create the required number of UAVs
            for i = 1:self.count
                self.uav{i} = self.GetModel(['uav',num2str(i)]);
                % Random spawn
                self.uav{i}.base = se3(se2((2 * rand()-1) * self.flightVolume(1)/2 ...
                                         , (2 * rand()-1) * self.flightVolume(2)/2 ...
                                         , (2 * rand()-1) * 2 * pi));
                 % Plot 3D model
                plot3d(self.uav{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0);
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
        % Move each of the UAVs forward and rotate some rotate value around
        % the z axis
        function PlotSingleRandomStep(self)
            for index = 1:self.count
                % Move Forward
                self.uav{index}.base = self.uav{index}.base * se3(se2(0.2, 0, 0));
                animate(self.uav{index},0);
                % Move Up (or down)
                self.uav{index}.base = self.uav{index}.base *  transl(0,0, (rand - 0.5) * 0.2);
                animate(self.uav{index},0);
                
                % Turn randomly
                self.uav{index}.base(1:3,1:3) = self.uav{index}.base(1:3,1:3) *  rotz((rand-0.5) * 30 * pi/180);
                animate(self.uav{index},0);                

                % If outside workspace rotate back around
                if self.uav{index}.base(1,4) < self.workspaceDimensions(1) ...
                || self.workspaceDimensions(2) < self.uav{index}.base(1,4) ...
                || self.uav{index}.base(2,4) < self.workspaceDimensions(3) ...
                || self.workspaceDimensions(4) < self.uav{index}.base(2,4) ...
                || self.uav{index}.base(3,4) < self.workspaceDimensions(5) ...
                || self.workspaceDimensions(6) < self.uav{index}.base(3,4)
                    self.uav{index}.base = self.uav{index}.base * se3(se2(-0.2, 0, 0)) * se3(se2(0, 0, pi));
                end
                
                % Move up since too low
                if self.uav{index}.base(3,4) < self.workspaceDimensions(5)
                    self.uav{index}.base = self.uav{index}.base * transl(0,0,0.2);
                end

                % Move down since too high
                if self.workspaceDimensions(6) < self.uav{index}.base(3,4)
                    self.uav{index}.base = self.uav{index}.base * transl(0,0,-0.2);
                end
            end
            % Do the drawing once for each interation for speed
            drawnow();
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
    end
    
    methods (Static)
        %% GetModel
        function model = GetModel(name)
            if nargin < 1
                name = 'ParrotQuadrotor';
            end
            [faceData,vertexData] = plyread('Parrot_ARDrone_Quadrotor.ply','tri');
            L1 = Link('alpha',0,'a',1,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,2) = vertexData(:,2);
            model.points = {vertexData,[]};
        end
    end    
end

