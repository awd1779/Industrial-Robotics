close all; clear all;



%% Environment Setup

workspace = [-2.5 2.5 -2.5 2.5 -0.05 2.5];

tablePosition = [0 2 0];
tablePose = transl(tablePosition);
tableMesh_h = PlaceObject('table_d.ply');
tableVertices = get(tableMesh_h,'Vertices');
tableTransformedVertices = [tableVertices,ones(size(tableVertices,1),1)] * tablePose';
set(tableMesh_h,'Vertices',tableTransformedVertices(:,1:3));
camlight;
scale = 0.5;
axis equal

hold on;

basePosition = [-0.11 2 0];
basePose = transl(basePosition);
baseMesh_h = PlaceObject('dishwasherBottom_d.ply');
baseVertices = get(baseMesh_h,'Vertices');
baseTransformedVertices = [baseVertices,ones(size(baseVertices,1),1)] * basePose';
set(baseMesh_h,'Vertices',baseTransformedVertices(:,1:3));

rackPosition = [-0.13 1.92 1];
rackPose = transl(rackPosition);
rackMesh_h = PlaceObject('dishrack_d.ply');
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

% setting up the robot

%Setup UR3
moto=motoman(false).model;
moto.base=transl(0.8,0,0);
q = zeros(1,6);
% q =[deg2rad(90) deg2rad(15) deg2rad(-60) deg2rad(50) deg2rad(90) 0];  %Initial position of UR3 to attain elbow down configuration
moto.plot3d(q);
moto.delay=0;

% Surface environment setup
surf([2.5,2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[2.5,2.5;0,0],'CData',imread('kitchen2.jpg'),'FaceColor','texturemap');
surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');






