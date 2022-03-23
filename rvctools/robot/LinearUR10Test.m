clear L
% clc
% close all

%           th     d           a        alpha
L(1) = Link([pi    0           0        pi/2    1]); % PRISMATIC Link
L(2) = Link([0     0.1697      0        -pi/2   0]);
L(3) = Link([0     0.176       0.6129   -pi     0]);
L(4) = Link([0     0.12781     0.5716	pi      0]);
L(5) = Link([0     0.1157      0        -pi/2	0]);
L(6) = Link([0     0.1157      0        -pi/2	0]);
L(7) = Link([0     0           0        0       0]);

L(1).qlim = [-0.45 0.45];
L(2).qlim = [-360 360]*pi/180;
L(3).qlim = [-360 360]*pi/180;
L(4).qlim = [-360 360]*pi/180;
L(5).qlim = [-360 360]*pi/180;
L(6).qlim = [-360 360]*pi/180;
L(7).qlim = [-360 360]*pi/180;

L(3).offset = -pi/2;
L(5).offset = -pi/2;

qz = [0 0 0 0 0 0 0];

LinearUR10 = SerialLink(L,'name','LinearUR10');
LinearUR10.plotopt = {'workspace', [-2 2 -2 2 -0.3 2],'scale',0.3};

LinearUR10.base = LinearUR10.base * trotx(pi/2);