function robot = DensoVM6083(name,concretePrintParams)
if nargin < 2
    concretePrintParams = [];
    if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['DensoVM6083G',datestr(now,'yyyymmddTHHMMSSFFF')];
    end
end
arm = load('DensoVM6083.mat');

L1=Link('alpha',-pi/2,'a',0.180, 'd',0.475, 'offset',0, 'qlim',[deg2rad(-170), deg2rad(170)]);
L2=Link('alpha',0,'a',0.385, 'd',0, 'offset',-pi/2, 'qlim',[deg2rad(-90), deg2rad(135)]);
L3=Link('alpha',pi/2,'a',-0.100, 'd',0, 'offset',pi/2, 'qlim',[deg2rad(-80), deg2rad(165)]);
L4=Link('alpha',-pi/2,'a',0, 'd',0.329+0.116, 'offset',0, 'qlim',[deg2rad(-185), deg2rad(185)]);
L5=Link('alpha',pi/2,'a',0, 'd',0, 'offset',0, 'qlim',[deg2rad(-120), deg2rad(120)]);
L6=Link('alpha',0,'a',0, 'd',0.09, 'offset',0, 'qlim',[deg2rad(-360), deg2rad(360)]);

robot = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);

% Use glyphs to draw robot, don't display the name
robot.plotopt = {'nojoints', 'noname', 'noshadow', 'nowrist'}; %

for linkIndex = 0:robot.n
%     [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['J',num2str(linkIndex),'.ply'],'tri');
    robot.faces{linkIndex+1} = arm.shapeModel(linkIndex+1).face;
    robot.points{linkIndex+1} = arm.shapeModel(linkIndex+1).vertex;
end

%% Switch Case for nozzle selection for concretePrint
if ~isempty(concretePrintParams)
%     nozzleSelect = 0;  % <--- Edit for GUI interaction
    if ~isfield(concretePrintParams,'nozzleSelect')
        concretePrintParams.nozzleSelect = 0;
    end
    switch (concretePrintParams.nozzleSelect)
        case (0)
            nozzle = '10mmCirExtruder';
        case (1)
            nozzle = '20mmCirExtruder';
        case (2)
            nozzle = '10mmSquExtruder';
        case (3)
            nozzle = '20mmSquExtruder';
    end
    hold on
    [faceData, vertexData, plyData{7}] = plyread([nozzle,'.ply'],'tri');
    robot.faces{7} = faceData;
    robot.points{7} = vertexData;
    q = [0,0,pi/2,0,pi/9,0];
else % Normal robot
    q = [0,0,pi/2,0,0,0];
end

%% Plot and color
robot.delay = 0;
% Check if there is a robot with this name, otherwise plot one 
if isempty(findobj('Tag', robot.name))
    robot.plot3d(q);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end            
end
robot.delay = 0;
handles = findobj('Tag', robot.name);
h = get(handles,'UserData');  
h.link(1).Children.FaceColor = [0.4,0.4,1];
h.link(2).Children.FaceColor = [1, 1, 0.9];
h.link(3).Children.FaceColor = [1, 1, 0.9];
h.link(4).Children.FaceColor = [1, 1, 0.9];
h.link(5).Children.FaceColor = [220 221 226]/255;
h.link(6).Children.FaceColor = [220 221 226]/255;
h.link(7).Children.FaceColor = [220 221 226]/255;

end
