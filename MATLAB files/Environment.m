classdef Environment < handle
    properties
        object;
        
        model;
        model_h;
        modelPose;
    end
    
    methods
%% Class constructor
        function self = Environment()
            %self.PlotObject();
        end
%% 
        function PlotObject(self, FileName, modelPose)
            % Read the PLY file
            A = string(FileName);            
            [faceData, vertexData, plyData] = plyread(A,'tri');
            
            % Get vertex count
            modelVertexCount = size(vertexData, 1);
            
            % Move center point to origin
            midPoint    = sum(vertexData)/modelVertexCount;
            modelVerts  = vertexData - repmat(midPoint, modelVertexCount, 1);
            
            % Scale the colors to be [0,1]
            vertexColors = [plyData.vertex.red, plyData.vertex.green, plyData.vertex.blue] / 255;
            
            % Plot the trisurf
            self.model_h = trisurf(faceData, modelVerts(:,1), modelVerts(:,2), modelVerts(:,3), 'FaceVertexCData', vertexColors, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            
            % Create a transform for model pose
            self.modelPose = eye(4)*transl(0,0,-1);
            updatePoints = [modelPose * [modelVerts, ones(modelVertexCount, 1)]']';
            
            % Update the vertices
            self.model_h.Vertices = updatePoints(:,1:3);
            
   
        end
    end
end

