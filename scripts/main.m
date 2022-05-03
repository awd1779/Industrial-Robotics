close all;

robot = motoman(false);

% name = "MOTOMAN HC10DTP";
% 
% L1 = Link('d',0.275,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
% L2 = Link('d',0,'a',0.7,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
% L3 = Link('d',0,'a',0.5,'alpha',pi,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
% L4 = Link('d',0,'a',0.162,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
% L5 = Link('d',0.170,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
% L6 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
% 
% % L1 = Link('d',0.0892,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
% % L2 = Link('d',0.1357,'a',0.425,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
% % L3 = Link('d',0.1197,'a',0.39243,'alpha',pi,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
% % L4 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
% % L5 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
% % L6 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
% % 
% robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', name);
% 
% workspace = [-2 2 -2 2 0 2];
q = [0, 0, 0, 0, 0, 0];
% scale = 0.5;
% 
% robot.plot(q,'workspace',workspace,'scale',scale)
% robot.teach()
robot.model.animate(q)
robot.model.teach()

% mdl_puma560

% p560.plot(q)