%% Rosbag
clc;
bag = rosbag('a1.bag');
bagselect1 = select(bag, 'Topic', '/joint_states');

msgs = readMessages(bagselect1);
qMatrix = ones(6003, 6);

for i = 1:6003
    qMatrix(i,:) = msgs{i,1}.Position';
end

%%
MintUR3 = UR3(eye(4));
MintUR3.PlotAndColourRobot(qMatrix(1,:));
for i = 1:10:6003
    qq = qMatrix(i,:);
    MintUR3.model.animate(qq);
    drawnow();
end