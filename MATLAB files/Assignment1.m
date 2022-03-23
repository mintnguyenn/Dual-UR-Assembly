clear; clc; clf;

L = log4matlab('logFileName1.log');

%% Plot the objects
% Plot the fence
Fence = Environment();
Fence.PlotObject('fence2.ply', eye(4)*transl(0,0,-0.25));

hold on;
% Plot the table
Table = Environment();
Table.PlotObject('table1.ply', eye(4));

% Plot the lab supervisor
LabSupervisor = Environment();
LabSupervisor.PlotObject('labsupervisor.ply', eye(4)*transl(-3,2,0.2));

% Plot the emergency button
Button = Environment();
Button.PlotObject('estop1.ply',transl(-2,1.3,0.26));

% Plot the fire extinguisher
Fire = Environment();
Fire.PlotObject('fire.ply',transl(0,0,-0.5));

% Plot the UR3
qDefaultUR3 = [0 -pi/2 pi/2 -pi/2 -pi/2 0];
MintUR3 = UR3(eye(4)*transl(0.5,0,0.25));
MintUR3.PlotAndColourRobot(qDefaultUR3);

% Plot the Linear UR5
qDefaultUR5 = [-0.3 0 -pi/4 pi/4 0 -pi/2 0];
MintUR5 = UR5Linear(false);

hold on;

% Plot the Bricks
Brick1 = Brick(trotz(-pi/2) * transl(0   ,  0, 0.305) * trotx(pi));
Brick2 = Brick(trotz(-pi/2) * transl(0   ,  0.2, 0.305) * trotx(pi));
Brick3 = Brick(trotz(-pi/2) * transl(0   , -0.2, 0.305) * trotx(pi));
Brick4 = Brick(trotz(-pi/2) * transl(0.4 ,    0, 0.305) * trotx(pi));
Brick5 = Brick(trotz(-pi/2) * transl(0.4 ,  0.2, 0.305) * trotx(pi));
Brick6 = Brick(trotz(-pi/2) * transl(0.4 , -0.2, 0.305) * trotx(pi));
Brick7 = Brick(trotz(-pi/2) * transl(-0.4,    0, 0.305) * trotx(pi));
Brick8 = Brick(trotz(-pi/2) * transl(-0.4,  0.2, 0.305) * trotx(pi));
Brick9 = Brick(trotz(-pi/2) * transl(-0.4, -0.2, 0.305) * trotx(pi));

%% Move brick 1 if the pose is changed
input("Press Enter to start");
pause(3);
if ~isequal(Brick1.brickPose, trotz(-pi/2) * transl(0,0,0.305) * trotx(pi))
    qPrepareUR5 = (MintUR5.model.ikcon(rt2tr(t2r(MintUR5.model.fkine(qDefaultUR5)),(Brick1.brickPose(1:end-1,4)+[0;0;0.2])),qDefaultUR5));
    qDefaultToPrepareUR5 = jtraj(qDefaultUR5, qPrepareUR5, 50);

    qPickUR5 = (MintUR5.model.ikcon(rt2tr(t2r(MintUR5.model.fkine(qPrepareUR5)),(Brick1.brickPose(1:end-1,4)+[0;0;0.1])),qPrepareUR5));
    qPrepareToPickUR5 = jtraj(qPrepareUR5, qPickUR5, 50);

    qDropUR5 = MintUR5.model.ikcon(r2t(t2r(MintUR5.model.fkine(qPickUR5))) + (transl(0,0,0.405)) - eye(4), qPickUR5);
    qPickToDropUR5 = jtraj(qPickUR5, qDropUR5, 50);

    qDropToDefaultUR5 = jtraj(qDropUR5, qDefaultUR5, 50);

    % Move to prepare position
    for i = 1:50
        pUR5 = qDefaultToPrepareUR5(i,:);
        MintUR5.model.animate(pUR5);
        drawnow();
    end

    % Move to pick up position
    for i = 1:50
        pUR5 = qPrepareToPickUR5(i,:);
        MintUR5.model.animate(pUR5);
        drawnow();
    end

    % Move to drop-off position
    for i = 1:50 
        pUR5 = qPickToDropUR5(i,:);
        MintUR5.model.animate(pUR5);
        drawnow();
    
        Brick1.Move(MintUR5.model.fkine(qPickToDropUR5(i,:))+[0,0,0,0;0,0,0,0;0,0,0,-0.1;0,0,0,0]);
    end

    % Move back to default
    for i = 1:50
        pUR5 = qDropToDefaultUR5(i,:);
        MintUR5.model.animate(pUR5);
        drawnow();
    end
end
    
%% Move brick 2 and 3 to brick 1
% Trajectory UR3 - brick 2

qPrepareUR3       = (MintUR3.model.ikcon(rt2tr(t2r(MintUR3.model.fkine(qDefaultUR3)),(Brick2.brickPose(1:end-1,4)+[0;0;0.2])),qDefaultUR3));
qDefaultToPrepare = jtraj(qDefaultUR3, qPrepareUR3, 50);

qPickUR3       = (MintUR3.model.ikcon(rt2tr(t2r(MintUR3.model.fkine(qPrepareUR3)),(Brick2.brickPose(1:end-1,4)+[0;0;0.0])),qPrepareUR3));

% Logging
A = rt2tr(t2r(MintUR3.model.fkine(qPrepareUR3)),(Brick2.brickPose(1:end-1,4)+[0;0;0]));
B = MintUR3.model.fkine(qPickUR3);
C = 1000*abs(A(1:3,4)-B(1:3,4));
L.mlog = {L.DEBUG,'Brick 2',['The transform is',L.MatrixToString(A)]};
L.mlog = {L.DEBUG,'Fkine qPick2',['The transform is',L.MatrixToString(B)]};
L.mlog = {L.DEBUG,'Error Brick 2 (in mm)',['The transform is',L.MatrixToString(C)]};

qPrepareToPick = jtraj(qPrepareUR3, qPickUR3, 50);

qDropUR3 = MintUR3.model.ikcon(r2t(t2r(MintUR3.model.fkine(qPickUR3))) + (transl(0.135,0,0.305)) - eye(4), qPickUR3);
qPickToDrop = jtraj(qPickUR3, qDropUR3, 50);

qDropToDefault = jtraj(qDropUR3, qDefaultUR3, 50);

% Trajectory UR5 - brick 4
qPrepareUR5 = (MintUR5.model.ikcon(rt2tr(t2r(MintUR5.model.fkine(qDefaultUR5)),(Brick3.brickPose(1:end-1,4)+[0;0;0.3])),qDefaultUR5));
qDefaultToPrepareUR5 = jtraj(qDefaultUR5, qPrepareUR5, 50);

qPickUR5 = (MintUR5.model.ikcon(rt2tr(t2r(MintUR5.model.fkine(qPrepareUR5)),(Brick3.brickPose(1:end-1,4)+[0;0;0.1])),qPrepareUR5));

% Logging
A = rt2tr(t2r(MintUR5.model.fkine(qPrepareUR5)),(Brick3.brickPose(1:end-1,4)+[0;0;0.1]));
B = MintUR5.model.fkine(qPickUR5);
C = 1000*abs(A(1:3,4)-B(1:3,4));
L.mlog = {L.DEBUG,'Brick 3',['The transform is',L.MatrixToString(A)]};
L.mlog = {L.DEBUG,'Fkine qPick3',['The transform is',L.MatrixToString(B)]};
L.mlog = {L.DEBUG,'Error Brick 3 (in mm)',['The transform is',L.MatrixToString(C)]};

qPrepareToPickUR5 = jtraj(qPrepareUR5, qPickUR5, 50);

qDropUR5 = MintUR5.model.ikcon(r2t(t2r(MintUR5.model.fkine(qPickUR5))) + (transl(-0.135,0,0.405)) - eye(4), qPickUR5);
qPickToDropUR5 = jtraj(qPickUR5, qDropUR5, 50);

qDropToDefaultUR5 = jtraj(qDropUR5, qDefaultUR5, 50);

% Move to prepare position
for i = 1:50
    pUR3 = qDefaultToPrepare(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    pUR5 = qDefaultToPrepareUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

% Move to pick up position
for i = 1:50
    pUR3 = qPrepareToPick(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    pUR5 = qPrepareToPickUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

% Move to drop-off position
for i = 1:50
    pUR3 = qPickToDrop(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    Brick2.Move(MintUR3.model.fkine(qPickToDrop(i,:)));
    
    pUR5 = qPickToDropUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
    
    Brick3.Move(MintUR5.model.fkine(qPickToDropUR5(i,:))+[0,0,0,0;0,0,0,0;0,0,0,-0.1;0,0,0,0]);
end

% Move back to default
for i = 1:50
    pUR3 = qDropToDefault(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    pUR5 = qDropToDefaultUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

%% Move brick 4 by Linear UR5
% Trajectory UR5 - brick 4
qPrepareUR5 = (MintUR5.model.ikcon(rt2tr(t2r(MintUR5.model.fkine(qDefaultUR5)),(Brick4.brickPose(1:end-1,4)+[0;0;0.3])),qDefaultUR5));
qDefaultToPrepareUR5 = jtraj(qDefaultUR5, qPrepareUR5, 50);

qPickUR5 = (MintUR5.model.ikcon(rt2tr(t2r(MintUR5.model.fkine(qPrepareUR5)),(Brick4.brickPose(1:end-1,4)+[0;0;0.1])),qPrepareUR5));

% Logging
A = rt2tr(t2r(MintUR5.model.fkine(qPrepareUR5)),(Brick4.brickPose(1:end-1,4)+[0;0;0.1]));
B = MintUR5.model.fkine(qPickUR5);
C = 1000*abs(A(1:3,4)-B(1:3,4));
L.mlog = {L.DEBUG,'Brick 4',['The transform is',L.MatrixToString(A)]};
L.mlog = {L.DEBUG,'Fkine qPick4',['The transform is',L.MatrixToString(B)]};
L.mlog = {L.DEBUG,'Error Brick 4 (in mm)',['The transform is',L.MatrixToString(C)]};

qPrepareToPickUR5 = jtraj(qPrepareUR5, qPickUR5, 50);

qDropUR5 = MintUR5.model.ikcon(r2t(t2r(MintUR5.model.fkine(qPickUR5))) + (transl(0,0,0.475)) - eye(4), qPickUR5);
qPickToPrepareUR5 = jtraj(qPickUR5, qPrepareUR5, 50);
qPrepareToDropUR5 = jtraj(qPrepareUR5, qDropUR5, 50);

qDropToDefaultUR5 = jtraj(qDropUR5, qDefaultUR5, 50);

% Move to prepare position
for i = 1:50
    pUR5 = qDefaultToPrepareUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

% Move to pick up position
for i = 1:50
    pUR5 = qPrepareToPickUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

% Move back to prepare position
for i = 1:50 
    pUR5 = qPickToPrepareUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
    
    Brick4.Move(MintUR5.model.fkine(qPickToPrepareUR5(i,:))+[0,0,0,0;0,0,0,0;0,0,0,-0.1;0,0,0,0]);
end

% Move to drop-off position
for i = 1:50 
    pUR5 = qPrepareToDropUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
    
    Brick4.Move(MintUR5.model.fkine(qPrepareToDropUR5(i,:))+[0,0,0,0;0,0,0,0;0,0,0,-0.1;0,0,0,0]);
end

% Move back to default
for i = 1:50
    pUR5 = qDropToDefaultUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

%% Move brick 8 -> 2, 6 -> 3
% Trajectory UR3 - brick 8
pause(1);
qPrepareUR3       = (MintUR3.model.ikcon(rt2tr(t2r(MintUR3.model.fkine(qDefaultUR3)),(Brick8.brickPose(1:end-1,4)+[0;0;0.2])),qDefaultUR3));
qDefaultToPrepare = jtraj(qDefaultUR3, qPrepareUR3, 50);

qPickUR3       = (MintUR3.model.ikcon(rt2tr(t2r(MintUR3.model.fkine(qPrepareUR3)),(Brick8.brickPose(1:end-1,4)+[0;0;0.0])),qPrepareUR3));

% Logging
A = rt2tr(t2r(MintUR3.model.fkine(qPrepareUR3)),(Brick8.brickPose(1:end-1,4)+[0;0;0]));
B = MintUR3.model.fkine(qPickUR3);
C = 1000*abs(A(1:3,4)-B(1:3,4));
L.mlog = {L.DEBUG,'Brick 8',['The transform is',L.MatrixToString(A)]};
L.mlog = {L.DEBUG,'Fkine qPick8',['The transform is',L.MatrixToString(B)]};
L.mlog = {L.DEBUG,'Error Brick 8 (in mm)',['The transform is',L.MatrixToString(C)]};

qPrepareToPick = jtraj(qPrepareUR3, qPickUR3, 50);
qPickToPrepare = jtraj(qPickUR3, qPrepareUR3, 50);

qDropUR3 = MintUR3.model.ikcon(r2t(t2r(MintUR3.model.fkine(qPickUR3))) + (transl(0.135,0,0.375)) - eye(4), qPickUR3);
qPrepareToDrop = jtraj(qPrepareUR3, qDropUR3, 50);

qDropToDefault = jtraj(qDropUR3, qDefaultUR3, 50);

% Trajectory UR5 - brick 4
qPrepareUR5 = (MintUR5.model.ikcon(rt2tr(t2r(MintUR5.model.fkine(qDefaultUR5)),(Brick6.brickPose(1:end-1,4)+[0;0;0.3])),qDefaultUR5));
qDefaultToPrepareUR5 = jtraj(qDefaultUR5, qPrepareUR5, 50);

qPickUR5 = (MintUR5.model.ikcon(rt2tr(t2r(MintUR5.model.fkine(qPrepareUR5)),(Brick6.brickPose(1:end-1,4)+[0;0;0.1])),qPrepareUR5));

% Logging
A = rt2tr(t2r(MintUR5.model.fkine(qPrepareUR5)),(Brick6.brickPose(1:end-1,4)+[0;0;0.1]));
B = MintUR5.model.fkine(qPickUR5);
C = 1000*abs(A(1:3,4)-B(1:3,4));
L.mlog = {L.DEBUG,'Brick 6',['The transform is',L.MatrixToString(A)]};
L.mlog = {L.DEBUG,'Fkine qPick6',['The transform is',L.MatrixToString(B)]};
L.mlog = {L.DEBUG,'Error Brick 6 (in mm)',['The transform is',L.MatrixToString(C)]};

qPrepareToPickUR5 = jtraj(qPrepareUR5, qPickUR5, 50);
qPickToPrepareUR5 = jtraj(qPickUR5, qPrepareUR5, 50);

qDropUR5 = MintUR5.model.ikcon(r2t(t2r(MintUR5.model.fkine(qPickUR5))) + (transl(-0.135,0,0.475)) - eye(4), qPickUR5);
qPrepareToDropUR5 = jtraj(qPrepareUR5, qDropUR5, 50);

qDropToDefaultUR5 = jtraj(qDropUR5, qDefaultUR5, 50);

% Move to prepare position
for i = 1:50
    pUR3 = qDefaultToPrepare(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    pUR5 = qDefaultToPrepareUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

% Move to pick up position
for i = 1:50
    pUR3 = qPrepareToPick(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    pUR5 = qPrepareToPickUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

% Move back to prepare position
for i = 1:50
    pUR3 = qPickToPrepare(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    Brick8.Move(MintUR3.model.fkine(qPickToPrepare(i,:)));
    
    pUR5 = qPickToPrepareUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
    
    Brick6.Move(MintUR5.model.fkine(qPickToPrepareUR5(i,:))+[0,0,0,0;0,0,0,0;0,0,0,-0.1;0,0,0,0]);
end

% Move to drop-off position
for i = 1:50
    pUR3 = qPrepareToDrop(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    Brick8.Move(MintUR3.model.fkine(qPrepareToDrop(i,:)));
    
    pUR5 = qPrepareToDropUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
    
    Brick6.Move(MintUR5.model.fkine(qPrepareToDropUR5(i,:))+[0,0,0,0;0,0,0,0;0,0,0,-0.1;0,0,0,0]);
end

% Move back to default
for i = 1:50
    pUR3 = qDropToDefault(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    pUR5 = qDropToDefaultUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

%% Move brick 5 -> 2, 9 -> 3
% Trajectory UR3 - brick 5
pause(1);
qPrepareUR3       = (MintUR3.model.ikcon(rt2tr(t2r(MintUR3.model.fkine(qDefaultUR3)),(Brick5.brickPose(1:end-1,4)+[0;0;0.2])),qDefaultUR3));
qDefaultToPrepare = jtraj(qDefaultUR3, qPrepareUR3, 50);

qPickUR3       = (MintUR3.model.ikcon(rt2tr(t2r(MintUR3.model.fkine(qPrepareUR3)),(Brick5.brickPose(1:end-1,4)+[0;0;0.0])),qPrepareUR3));

% Logging
A = rt2tr(t2r(MintUR3.model.fkine(qPrepareUR3)),(Brick5.brickPose(1:end-1,4)+[0;0;0]));
B = MintUR3.model.fkine(qPickUR3);
C = 1000*abs(A(1:3,4)-B(1:3,4));
L.mlog = {L.DEBUG,'Brick 5',['The transform is',L.MatrixToString(A)]};
L.mlog = {L.DEBUG,'Fkine qPick5',['The transform is',L.MatrixToString(B)]};
L.mlog = {L.DEBUG,'Error Brick 5 (in mm)',['The transform is',L.MatrixToString(C)]};

qPrepareToPick = jtraj(qPrepareUR3, qPickUR3, 50);
qPickToPrepare = jtraj(qPickUR3, qPrepareUR3, 50);

qDropUR3 = MintUR3.model.ikcon(r2t(t2r(MintUR3.model.fkine(qPickUR3))) + (transl(0.135,0,0.445)) - eye(4), qPickUR3);
qPrepareToDrop = jtraj(qPrepareUR3, qDropUR3, 50);

qDropToDefault = jtraj(qDropUR3, qDefaultUR3, 50);

% Trajectory UR5 - brick 9
qPrepareUR5 = (MintUR5.model.ikcon(rt2tr(t2r(MintUR5.model.fkine(qDefaultUR5)),(Brick9.brickPose(1:end-1,4)+[0;0;0.3])),qDefaultUR5));
qDefaultToPrepareUR5 = jtraj(qDefaultUR5, qPrepareUR5, 50);

qPickUR5 = (MintUR5.model.ikcon(rt2tr(t2r(MintUR5.model.fkine(qPrepareUR5)),(Brick9.brickPose(1:end-1,4)+[0;0;0.1])),qPrepareUR5));

% Logging
A = rt2tr(t2r(MintUR5.model.fkine(qPrepareUR5)),(Brick9.brickPose(1:end-1,4)+[0;0;0.1]));
B = MintUR5.model.fkine(qPickUR5);
C = 1000*abs(A(1:3,4)-B(1:3,4));
L.mlog = {L.DEBUG,'Brick 9',['The transform is',L.MatrixToString(A)]};
L.mlog = {L.DEBUG,'Fkine qPick9',['The transform is',L.MatrixToString(B)]};
L.mlog = {L.DEBUG,'Error Brick 9 (in mm)',['The transform is',L.MatrixToString(C)]};

qPrepareToPickUR5 = jtraj(qPrepareUR5, qPickUR5, 50);

qPickToPrepareUR5 = jtraj(qPickUR5, qPrepareUR5, 50);

qDropUR5 = MintUR5.model.ikcon(r2t(t2r(MintUR5.model.fkine(qPickUR5))) + (transl(-0.135,0,0.545)) - eye(4), qPickUR5);
qPrepareToDropUR5 = jtraj(qPrepareUR5, qDropUR5, 50);

qDropToDefaultUR5 = jtraj(qDropUR5, qDefaultUR5, 50);

% Move to prepare position
for i = 1:50
    pUR3 = qDefaultToPrepare(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    pUR5 = qDefaultToPrepareUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

% Move to pick up position
for i = 1:50
    pUR3 = qPrepareToPick(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    pUR5 = qPrepareToPickUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

% Move back to prepare position
for i = 1:50
    pUR3 = qPickToPrepare(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    Brick5.Move(MintUR3.model.fkine(qPickToPrepare(i,:)));
    
    pUR5 = qPickToPrepareUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
    
    Brick9.Move(MintUR5.model.fkine(qPickToPrepareUR5(i,:))+[0,0,0,0;0,0,0,0;0,0,0,-0.1;0,0,0,0]);
end

% Move to drop-off position
for i = 1:50
    pUR3 = qPrepareToDrop(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    Brick5.Move(MintUR3.model.fkine(qPrepareToDrop(i,:)));
    
    pUR5 = qPrepareToDropUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
    
    Brick9.Move(MintUR5.model.fkine(qPrepareToDropUR5(i,:))+[0,0,0,0;0,0,0,0;0,0,0,-0.1;0,0,0,0]);
end

% Move back to default
for i = 1:50
    pUR3 = qDropToDefault(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    pUR5 = qDropToDefaultUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

%% Move brick 7 by Linear UR5
% Trajectory UR5 - brick 4
qPrepareUR5 = (MintUR5.model.ikcon(rt2tr(t2r(MintUR5.model.fkine(qDefaultUR5)),(Brick7.brickPose(1:end-1,4)+[0;0;0.3])),qDefaultUR5));
qDefaultToPrepareUR5 = jtraj(qDefaultUR5, qPrepareUR5, 50);

qPickUR5 = (MintUR5.model.ikcon(rt2tr(t2r(MintUR5.model.fkine(qPrepareUR5)),(Brick7.brickPose(1:end-1,4)+[0;0;0.1])),qPrepareUR5));

% Logging
A = rt2tr(t2r(MintUR5.model.fkine(qPrepareUR5)),(Brick7.brickPose(1:end-1,4)+[0;0;0.1]));
B = MintUR5.model.fkine(qPickUR5);
C = 1000*abs(A(1:3,4)-B(1:3,4));
L.mlog = {L.DEBUG,'Brick 7',['The transform is',L.MatrixToString(A)]};
L.mlog = {L.DEBUG,'Fkine qPick7',['The transform is',L.MatrixToString(B)]};
L.mlog = {L.DEBUG,'Error Brick 7 (in mm)',['The transform is',L.MatrixToString(C)]};

qPrepareToPickUR5 = jtraj(qPrepareUR5, qPickUR5, 50);

qDropUR5 = MintUR5.model.ikcon(r2t(t2r(MintUR5.model.fkine(qPickUR5))) + (transl(0,0,0.545)) - eye(4), qPickUR5);
qPickToPrepareUR5 = jtraj(qPickUR5, qPrepareUR5, 50);
qPrepareToDropUR5 = jtraj(qPrepareUR5, qDropUR5, 50);

qDropToDefaultUR5 = jtraj(qDropUR5, qDefaultUR5, 50);

% Move to prepare position
for i = 1:50
    pUR5 = qDefaultToPrepareUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

% Move to pick up position
for i = 1:50
    pUR5 = qPrepareToPickUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

% Move back to prepare position
for i = 1:50 
    pUR5 = qPickToPrepareUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
    
    Brick7.Move(MintUR5.model.fkine(qPickToPrepareUR5(i,:))+[0,0,0,0;0,0,0,0;0,0,0,-0.1;0,0,0,0]);
end

% Move to drop-off position
for i = 1:50 
    pUR5 = qPrepareToDropUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
    
    Brick7.Move(MintUR5.model.fkine(qPrepareToDropUR5(i,:))+[0,0,0,0;0,0,0,0;0,0,0,-0.1;0,0,0,0]);
end

% Move back to default
for i = 1:50
    pUR5 = qDropToDefaultUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

%% Return to HOME
qHomeUR5 = [-0.3 0 0 0 0 -pi/2 0];
qHomeUR3 = [0 -pi/2 0 -pi/2 -pi/2 0];

qGoHomeUR3 = jtraj(qDefaultUR3, qHomeUR3, 50);
qGoHomeUR5 = jtraj(qDefaultUR5, qHomeUR5, 50);

for i = 1:50
    pUR3 = qGoHomeUR3(i,:);
    MintUR3.model.animate(pUR3);
    drawnow();
    
    pUR5 = qGoHomeUR5(i,:);
    MintUR5.model.animate(pUR5);
    drawnow();
end

%% PointCLoud
hold on;

if (false)
    input('Press Enter to continue');
    MintUR3.PointCloud();
    [k1,av1] = convhull(MintUR3.pointCloud);
    disp(['The approximate volume is ', num2str(av1), ' m3']);
    Radius = nthroot((3*av1)/(4*pi),3);
    disp(['The radius is ', num2str(Radius), ' m']);
end