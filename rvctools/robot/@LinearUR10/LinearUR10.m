classdef LinearUR10 < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-2 2 -2 2 -0.3 2];   
    end
    
    methods%% Class for UR10 robot simulation
function self = LinearUR10()

%> Define the boundaries of the workspace

        
% robot = 
self.GetUR10Robot();
% robot = 
self.PlotAndColourRobot();%robot,workspace);
end

%% GetUR10Robot
% Given a name (optional), create and return a UR10 robot model
function GetUR10Robot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['LinearUR_10_',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end

    % Create the UR10 model mounted on a linear rail
    L(1) = Link([pi    0           0        pi/2    1]); % PRISMATIC Link
    L(2) = Link([0     0.1697      0        -pi/2   0]);
    L(3) = Link([0     0.176       0.6129   -pi     0]);
    L(4) = Link([0     0.12781     0.5716	pi      0]);
    L(5) = Link([0     0.1157      0        -pi/2	0]);
    L(6) = Link([0     0.1157      0        -pi/2	0]);
    L(7) = Link([0     0           0        0       0]);

    % Incorporate joint limits
    L(1).qlim = [-0.4 0.4];
    L(2).qlim = [-360 360]*pi/180;
    L(3).qlim = [-90 90]*pi/180;
    L(4).qlim = [-170 170]*pi/180;
    L(5).qlim = [-360 360]*pi/180;
    L(6).qlim = [-360 360]*pi/180;
    L(7).qlim = [-360 360]*pi/180;

    L(3).offset = -pi/2;
    L(5).offset = -pi/2;

    self.model = SerialLink(L,'name',name);
    
    % Rotate robot to the correct orientation
    self.model.base = self.model.base * trotx(pi/2) * troty(pi/2);
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR10Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end        
    end
end