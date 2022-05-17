close all; clear all;



%% Environment Setup

workspace = [-2.5 2.5 -2.5 2.5 -0.05 2.5];

tablePosition = [0 2 0];
tablePose = transl(tablePosition);
tableMesh_h = PlaceObject('dishwasherBottom_d.ply');
tableVertices = get(tableMesh_h,'Vertices');
tableTransformedVertices = [tableVertices,ones(size(tableVertices,1),1)] * tablePose';
set(tableMesh_h,'Vertices',tableTransformedVertices(:,1:3));
camlight;
% scale = 0.5;
axis equal

hold on;

basePosition = [1 2 0];
basePose = transl(basePosition);
baseMesh_h = PlaceObject('newTable.ply');
baseVertices = get(baseMesh_h,'Vertices');
baseTransformedVertices = [baseVertices,ones(size(baseVertices,1),1)] * basePose';
set(baseMesh_h,'Vertices',baseTransformedVertices(:,1:3));
% 
rackPosition = [0 1.92 1];
rackPose = transl(rackPosition);
rackMesh_h = PlaceObject('dishTray.ply');
rackVertices = get(rackMesh_h,'Vertices');
rackTransformedVertices = [rackVertices,ones(size(rackVertices,1),1)] * rackPose';
set(rackMesh_h,'Vertices',rackTransformedVertices(:,1:3));

lidPosition = [-0.11 1.8 1.5];
lidPose = transl(lidPosition);
lidMesh_h = PlaceObject('toplid_d.ply');
lidVertices = get(lidMesh_h,'Vertices');
lidTransformedVertices = [lidVertices,ones(size(lidVertices,1),1)] * lidPose';
set(lidMesh_h,'Vertices',lidTransformedVertices(:,1:3));

armPosition = [-0.13 0.5 0];
armPose = transl(armPosition);
armMesh_h = PlaceObject('smallTable_d.ply');
armVertices = get(armMesh_h,'Vertices');
armTransformedVertices = [armVertices,ones(size(armVertices,1),1)] * armPose';
set(armMesh_h,'Vertices',armTransformedVertices(:,1:3));

fencePosition = [0.5 -2 0];
fencePose = transl(fencePosition);
fenceMesh_h = PlaceObject('fence.ply');
fenceVertices = get(fenceMesh_h,'Vertices');
fenceTransformedVertices = [fenceVertices,ones(size(fenceVertices,1),1)] * fencePose';
set(fenceMesh_h,'Vertices',fenceTransformedVertices(:,1:3));

fence2Position = [-1 0 0];
fence2Pose = transl(fence2Position)  * trotz(pi/2) ;
fence2Mesh_h = PlaceObject('fence.ply');
fence2Vertices = get(fence2Mesh_h,'Vertices');
fence2TransformedVertices = [fence2Vertices,ones(size(fence2Vertices,1),1)] * fence2Pose';
set(fence2Mesh_h,'Vertices',fence2TransformedVertices(:,1:3));

fireExtPosition = [2 2 0];
fireExtPose = transl(fireExtPosition)  * trotz(pi/2) ;
fireExtMesh_h = PlaceObject('fireExtinguisher.ply');
fireExtVertices = get(fireExtMesh_h,'Vertices');
fireExtTransformedVertices = [fireExtVertices,ones(size(fireExtVertices,1),1)] * fireExtPose';
set(fireExtMesh_h,'Vertices',fireExtTransformedVertices(:,1:3));

% firePosition = [2 2 0.35];
% PutObject('fireExtinguisher.ply',firePosition);

firstAidPosition = [2.1 2.5 1.1];
firstAidPose = transl(firstAidPosition);
firstAidMesh_h = PlaceObject('firstAid.ply');
firstAidVertices = get(firstAidMesh_h,'Vertices');
firstAidTransformedVertices = [firstAidVertices,ones(size(firstAidVertices,1),1)] * firstAidPose';
set(firstAidMesh_h,'Vertices',firstAidTransformedVertices(:,1:3));

% firstAidPosition = [2.1 2.5 1.5];
% PutObject('firstAid.ply',firstAidPosition);

% eStopPosition = [1 2.2 1];
% PutObject('eStop.ply',eStopPosition);
eStopPosition = [1 2.2 0.95];
eStopPose = transl(eStopPosition);
eStopMesh_h = PlaceObject('eStop.ply');
eStopVertices = get(eStopMesh_h,'Vertices');
eStopTransformedVertices = [eStopVertices,ones(size(eStopVertices,1),1)] * eStopPose';
set(eStopMesh_h,'Vertices',eStopTransformedVertices(:,1:3));

% setting up the robot

startingPos = zeros(1,6);

moto=motoman(false).model;
moto.base=transl(0,0.5,0.4 );
q = zeros(1,6);
% q =[deg2rad(90) deg2rad(15) deg2rad(-60) deg2rad(50) deg2rad(90) 0];  %Initial position of UR3 to attain elbow down configuration
moto.plot3d(q);
moto.delay=0;
moto.teach();

%     robot.delay=0;
%     robot.plot3d(startingPos);
%     qStart=startingPos;
%       pose=transl([-0.5,2,1]);
%     qEnd=robot.ikine(pose);
%     steps=50;
%     qMatrix=jtraj(qStart,qEnd,steps);
%     robot.animate(qEnd);
%     for i=1:steps
%         robot.animate(qMatrix(i,:));
%         drawnow();
%         pause(0.05);
%     end

%Setup UR5


% Surface environment setup
surf([2.5,2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[2.5,2.5;0,0],'CData',imread('kitchen2.jpg'),'FaceColor','texturemap');
surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
% surf([2.5,2.5;2.5,2.5],[-0.3,0.3;-0.3,0.3],[0.5,0.5;2,2],'CData',imread('safety.jpg'),'FaceColor','texturemap');





