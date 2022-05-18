%% This function adds new simulated object into the environment when called

function putObject(object,pose)
%% Call the function by parsing in the name of .ply file and specifying Pose
[f,v,data] = plyread(object,'tri'); %triangle mesh
objectVertexCount = size(v,1);
midPoint = sum(v)/objectVertexCount;
objectVerts = v - repmat(midPoint,objectVertexCount,1);
objectPose = eye(4); % create the matrix 4x4 for partpose 
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
objectMesh_h =  trisurf(f,objectVerts(:,1),objectVerts(:,2), objectVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
forwardTR = makehgtform('translate',pose);

objectPose = objectPose*forwardTR; %update the location

updatedPoints = [objectPose * [objectVerts,ones(objectVertexCount,1)]']';  %update the location
objectMesh_h.Vertices = updatedPoints(:,1:3);
end