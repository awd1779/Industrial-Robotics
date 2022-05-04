close all; clear all;



%% Environment Setup

workspace = [-2.5 2.5 -2.5 2.5 -0.05 2.5];

tablePosition = [0 -2 0];
tablePose = transl(tablePosition);
tableMesh_h = PlaceObject('table2.ply');
tableVertices = get(tableMesh_h,'Vertices');
tableTransformedVertices = [tableVertices,ones(size(tableVertices,1),1)] * tablePose';
set(tableMesh_h,'Vertices',tableTransformedVertices(:,1:3));
camlight;
axis equal

hold on;
% Surface environment setup
surf([2.5,2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[2.5,2.5;0,0],'CData',imread('kitchen2.jpg'),'FaceColor','texturemap');
surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');






