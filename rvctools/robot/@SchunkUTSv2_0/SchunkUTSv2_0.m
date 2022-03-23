function r = SchunkUTSv2_0(name)
if nargin < 1
    % Create a unique name (ms timestamp after 1ms pause)
    pause(0.001);
    name = ['SchunkUTSv2_0',datestr(now,'yyyymmddTHHMMSSFFF')];
end
% For each link specify
% alpha  link twist angle (this affects the next section 
% a      link length
% d      link offset distance
% offset sets the starting angle position, needs to be considered for ALL future links
% qlim   Joint limits in radians

L1 = Link('alpha',-pi/2,'a',0,'d',-0.38,'offset',0,'qlim',[deg2rad(-117),deg2rad(117)])
L2 = Link('alpha',pi,'a',0.385,'d',0,'offset',pi/2,'qlim',[deg2rad(-115),deg2rad(115)])
L3 = Link('alpha',pi/2,'a',0,'d',0,'offset',-pi/2,'qlim',[deg2rad(-110),deg2rad(110)])
L4 = Link('alpha',pi/2,'a',0,'d',-0.445,'offset',0,'qlim',[deg2rad(-200),deg2rad(200)])
L5 = Link('alpha',-pi/2,'a',0,'d',0,'offset',0,'qlim',[deg2rad(-107),deg2rad(107)])
L6 = Link('alpha',pi,'a',0,'d',-0.2106,'offset',0,'qlim',[deg2rad(-200),deg2rad(200)])

r = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
r.base = trotz(-pi/2) * trotx(pi);

tr_x_p90 = makehgtform('xrotate',pi/2); tr_x_p90 = tr_x_p90(1:3,1:3);
tr_y_p90 = makehgtform('yrotate',pi/2); tr_y_p90 = tr_y_p90(1:3,1:3);
tr_z_p90 = makehgtform('zrotate',pi/2); tr_z_p90 = tr_z_p90(1:3,1:3);

tr_x_n90 = makehgtform('xrotate',-pi/2); tr_x_n90 = tr_x_n90(1:3,1:3);
tr_y_n90 = makehgtform('yrotate',-pi/2); tr_y_n90 = tr_y_n90(1:3,1:3);
tr_z_n90 = makehgtform('zrotate',-pi/2); tr_z_n90 = tr_z_n90(1:3,1:3);

arm = load('SchunkUTSv2_0_lowResolution.mat');
nozzle = load('NozzleNoSensor.mat');
plate = load('plate.mat');

% Shift around the models loaded from file
arm.shapeModel(1).vertex(:,3) = arm.shapeModel(1).vertex(:,3) - 0.18; 
arm.shapeModel(1).vertex = arm.shapeModel(1).vertex * tr_x_p90;
arm.shapeModel(1).vertex = arm.shapeModel(1).vertex + repmat([-0.09,0.09,0],size(arm.shapeModel(1).vertex,1),1);

arm.shapeModel(2).vertex = arm.shapeModel(2).vertex + repmat([-0.065,-0.3,-0.097],size(arm.shapeModel(2).vertex,1),1);
arm.shapeModel(2).vertex = arm.shapeModel(2).vertex * tr_y_p90 * tr_y_p90;

arm.shapeModel(3).vertex = arm.shapeModel(3).vertex * tr_z_p90;
arm.shapeModel(3).vertex = arm.shapeModel(3).vertex + repmat([-0.685,0.065,-0.055],size(arm.shapeModel(3).vertex,1),1);

arm.shapeModel(4).vertex = arm.shapeModel(4).vertex * tr_x_p90;
arm.shapeModel(4).vertex = arm.shapeModel(4).vertex * tr_y_n90^-1;
arm.shapeModel(4).vertex = arm.shapeModel(4).vertex + repmat([-0.685,-0.09,-0.160],size(arm.shapeModel(4).vertex,1),1);

arm.shapeModel(5).vertex = arm.shapeModel(5).vertex * tr_z_p90^-1;
arm.shapeModel(5).vertex = arm.shapeModel(5).vertex + repmat([0.702,-0.845+0.7916,-0.033],size(arm.shapeModel(5).vertex,1),1);

arm.shapeModel(6).vertex = arm.shapeModel(6).vertex * tr_z_n90;
arm.shapeModel(6).vertex = arm.shapeModel(6).vertex * tr_x_n90;
arm.shapeModel(6).vertex = arm.shapeModel(6).vertex + repmat([0,0.027,0.001],size(arm.shapeModel(6).vertex,1),1);

plate.vertex = plate.vertex * tr_z_p90;

for i = 1: r.n+1
    r.faces{i} = arm.shapeModel(i).face;
    r.points{i} = arm.shapeModel(i).vertex;
end

r.delay = 0;
% Check if there is a robot with this name, otherwise plot one 
if isempty(findobj('Tag', r.name))
    r.plot3d([0,0,pi/2,0,0,0]);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end            
end
r.delay = 0;
handles = findobj('Tag', r.name);
h = get(handles,'UserData');  
h.link(1).Children.FaceColor = [0.7,0.7,0.7];
h.link(2).Children.FaceColor = [1, 1, 0.8];
h.link(3).Children.FaceColor = [1, 1, 0.9];
h.link(4).Children.FaceColor = [1, 1, 0.9];
h.link(5).Children.FaceColor = [0.8,0.8,0.8];
h.link(6).Children.FaceColor = [0.8,0.8,0.8];
h.link(7).Children.FaceColor = [0.8,0.8,0.8];
