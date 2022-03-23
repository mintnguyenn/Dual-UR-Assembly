classdef Brick < handle
    properties 
        brick_h;
        brickPose;
        updatePoints;
        brickVerts;
        brickVertexCount;
    end
    
    methods
%% Class constructor
        function self = Brick(brickPose)
            self.brickPose = brickPose;
            
            self.PlaceBrick(self.brickPose);
            
        end
%% 
        function PlaceBrick(self, brickPose)
            % Read the PLY file           
            [faceData, vertexData, plyData] = plyread('brick.ply','tri');
            
            % Get vertex count
            self.brickVertexCount = size(vertexData, 1);
            
            % Move center point to origin
            midPoint    = sum(vertexData)/self.brickVertexCount;
            self.brickVerts  = vertexData - repmat(midPoint, self.brickVertexCount, 1);
            
            % Scale the colors to be [0,1]
            vertexColors = [plyData.vertex.red, plyData.vertex.green, plyData.vertex.blue] / 255;
            
            % Plot the trisurf
            self.brick_h = trisurf(faceData, self.brickVerts(:,1), self.brickVerts(:,2), self.brickVerts(:,3), 'FaceVertexCData', vertexColors, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            
            % Create a transform for model pose
            self.brickPose = brickPose;
            self.updatePoints = [self.brickPose * [self.brickVerts, ones(self.brickVertexCount, 1)]']';
            
            % Update the vertices
            self.brick_h.Vertices = self.updatePoints(:,1:3);
        end
        
        %%
        function Move(self, NewPose)
            self.brickPose = NewPose;
            self.updatePoints = [self.brickPose * [self.brickVerts, ones(self.brickVertexCount, 1)]']';
    
            self.brick_h.Vertices = self.updatePoints(:,1:3);
            drawnow();
            
        end
    end
end