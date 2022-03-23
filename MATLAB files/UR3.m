classdef UR3 < handle
    properties
        %> Robot model
        model;
    
        %>
        workspace = [-2 2 -2 2 -0.3 2];
        
        qJoint = zeros(1,6);
        pointCloud;
        distance;
        maxDistance = 0;
    end
    
    methods
        function self = UR3(base)    
            %> Define the boundaries of the workspace
            self.GetUR3Robot(base);
            % self.PlotAndColourRobot(self.q_joint);%robot,workspace);
        end

        %% GetUR3Robot
        % Given a name (optional), create and return a UR3 robot model
        function GetUR3Robot(self, baseUR3)
            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.001);
            name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];

            % Create the UR3 model mounted on a linear rail
            L(1) = Link('d',0.1519 ,'a',0       ,'alpha',pi/2 ,'offset',0,'qlim', [-2*pi,2*pi]);
            L(2) = Link('d',0      ,'a',-0.24365,'alpha',0    ,'offset',0,'qlim', [-2*pi,2*pi]);
            L(3) = Link('d',0      ,'a',-0.21325,'alpha',0    ,'offset',0,'qlim', [-2*pi,2*pi]);
            L(4) = Link('d',0.11235,'a',0       ,'alpha',pi/2 ,'offset',0,'qlim', [-2*pi,2*pi]);
            L(5) = Link('d',0.08535,'a',0       ,'alpha',-pi/2,'offset',0,'qlim', [-2*pi,2*pi]);
            L(6) = Link('d',0.0819 ,'a',0       ,'alpha',0    ,'offset',0,'qlim', [-2*pi,2*pi]);
 
            self.model = SerialLink(L,'name',name);

            % Rotate robot to the correct orientation
            self.model.base = baseUR3;
        end
        
        %% PlotAndColourRobot
        function PlotAndColourRobot(self, qJoint)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['U',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
            
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

            % Display robot
            self.model.plot3d(qJoint,'noarrow','workspace',self.workspace);
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

%% Pointcloud
        function PointCloud(self)
            stepRads = deg2rad(30);
            qlim = self.model.qlim;
% Don't need to worry about joint 6
            pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
            self.pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            tic

for q1 = qlim(1,1):stepRads:qlim(1,2)
    for q2 = qlim(2,1):stepRads:qlim(2,2)
        for q3 = qlim(3,1):stepRads:qlim(3,2)
            for q4 = qlim(4,1):stepRads:qlim(4,2)
                for q5 = qlim(5,1):stepRads:qlim(5,2)
                    % Don't need to worry about joint 6, just assume it=0
                    q6 = 0;

                        q = [q1,q2,q3,q4,q5,q6];
                        tr = self.model.fkine(q);
%                         self.distance = norm(tr(1:3,4)'-self.model.base(1:3,4)');
                        self.pointCloud(counter,:) = tr(1:3,4)';
%                         if self.distance > self.maxDistance
%                             self.maxDistance = self.distance;
%                         end
                        counter = counter + 1; 
                        if mod(counter/pointCloudeSize * 100,1) == 0
                            disp(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                        end
                end
            end
        end
    end
end

% Create a 3D model showing where the end effector can be over all these samples.  
plot3(self.pointCloud(:,1),self.pointCloud(:,2),self.pointCloud(:,3),'r.');
        end
        
    end
end