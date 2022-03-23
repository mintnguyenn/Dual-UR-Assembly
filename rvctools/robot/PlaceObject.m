%% PlaceObject
% Place ply file objects into the environment at a certain location
function [ mesh_h ] = PlaceObject( name , locations )
    [f,v,data] = plyread(name,'tri');

    if nargin < 2
        locations = [0,0,0];
    end

    % Scale the colours to be 0-to-1 (they are originally 0-to-255)            
    try
        try
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        catch
            vertexColours = [data.face.red, data.face.green, data.face.blue] / 255;        
        end
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        mesh_h = zeros(size(locations,1),1);
        for i = 1: size(locations,1)
            % Plot the Trisurf
            
%             mesh_h(i) = trisurf(f,v(:,1)+locations(i,1),v(:,2)+locations(i,2), v(:,3)+locations(i,3) ...
%                 ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            mesh_h(i) = trisurf(f,v(:,1)+locations(i,1),v(:,2)+locations(i,2), v(:,3)+locations(i,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
        end
    % Catch the case where there are no colours
    catch ME_1
        disp(ME_1);
    end
end