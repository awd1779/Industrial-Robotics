function [] = main()
    clf

    startingPos = zeros(1,6);
    robot = motoman(false);
    workspace = [-2,2,-2,2,0,2];
    
    
    axis equal
%     camlight
    robot.model.animate(zeros(1,6));
%     robot.model.teach
%     input('yo')
    
%     robot.model.plot(startingPos,'workspace',workspace,'floorlevel',0,'noarrow', 'scale', 0.8)
%     robot.model.teach
%     robot.model.delay = 0;
    hold on
%     input('a')

    centerpnt = [-0.5,0.5,0.25];
    side = 0.5;
    plotOptions.plotFaces = true;
    [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

    pickupDirtyPose = transl(.8,.8,0.2) * troty(pi/2);
    washingMachine = transl(-1,0,0.2) * troty(-pi/2);
    dropoffCleanPose = transl(.8,-.8,0.5)*troty(pi/2);

    elbowDownDishwasher = [pi, pi/4, -pi/2, pi/4, -pi/2, 0];
    elbowDownDrier = [0, pi/4, -pi/2, pi/4, -pi/2, 0];
    
    pickupDirtyWaypointQ = robot.model.ikcon(pickupDirtyPose * transl(0,0,-0.3),elbowDownDrier);
    pickupDirtyQ = robot.model.ikcon(pickupDirtyPose, elbowDownDrier);
    washingMachineWaypointQ = robot.model.ikcon(washingMachine * transl(0,0,-0.3), elbowDownDishwasher);
    washingMachineQ = robot.model.ikcon(washingMachine, elbowDownDishwasher);
    dropoffCleanWaypointQ = robot.model.ikcon(dropoffCleanPose * transl(0,0,-0.3), elbowDownDrier);
    dropoffCleanQ = robot.model.ikcon(dropoffCleanPose, elbowDownDrier);

%     steps = 150;
%     robotTraj = zeros(steps*7,6);
    
    
%     robotTraj(1:steps*0.5,:) = jtraj(startingPos, pickupDirtyWaypointQ, steps/2);
    robotTraj{1} = getTrajectory(startingPos, pickupDirtyWaypointQ);
%     robotTraj(steps*0.5+1:steps*1.5,:) = jtraj(pickupDirtyWaypointQ, pickupDirtyQ, steps);
    robotTraj{2} = getTrajectory(pickupDirtyWaypointQ, pickupDirtyQ);
%     robotTraj(steps*1.5+1:steps*2,:) = jtraj(pickupDirtyQ, pickupDirtyWaypointQ, steps/2);
    robotTraj{3} = getTrajectory(pickupDirtyQ, pickupDirtyWaypointQ);
%     robotTraj(steps*2+1:steps*2.5,:) = jtraj(pickupDirtyWaypointQ, washingMachineWaypointQ, steps/2);
    robotTraj{4} = getTrajectory(pickupDirtyWaypointQ, washingMachineWaypointQ);
%     robotTraj(steps*2.5+1:steps*3.5,:) = jtraj(washingMachineWaypointQ, washingMachineQ, steps);
    robotTraj{5} = getTrajectory(washingMachineWaypointQ, washingMachineQ);
%     robotTraj(steps*3.5+1:steps*4,:) = jtraj(washingMachineQ, washingMachineWaypointQ, steps/2);
    robotTraj{6} = getTrajectory(washingMachineQ, washingMachineWaypointQ);
%     robotTraj(steps*4+1:steps*4.5,:) = jtraj(washingMachineWaypointQ, dropoffCleanWaypointQ, steps/2);
    robotTraj{7} = getTrajectory(washingMachineWaypointQ, dropoffCleanWaypointQ);
%     robotTraj(steps*4.5+1:steps*5.5,:) = jtraj(dropoffCleanWaypointQ, dropoffCleanQ, steps);
    robotTraj{8} = getTrajectory(dropoffCleanWaypointQ, dropoffCleanQ);
%     robotTraj(steps*5.5+1:steps*6,:) = jtraj(dropoffCleanQ, dropoffCleanWaypointQ, steps/2);
    robotTraj{9} = getTrajectory(dropoffCleanQ, dropoffCleanWaypointQ);
%     robotTraj(steps*6+1:steps*7,:) = jtraj(dropoffCleanWaypointQ, startingPos, steps);
    robotTraj{10} = getTrajectory(dropoffCleanWaypointQ, startingPos);

    stop = false;
    for i = 1 : 10
        steps = size(robotTraj{i});
        for j = 1 : steps(1)
            if stop
                while(stop)
                end
            end
            if (IsCollision(robot.model,robotTraj{i}(j,:),faces,vertex,faceNormals,false))
                disp("COLLISION")

                qWaypoints = [robotTraj{i}(j,:); robotTraj{i}(size(robotTraj{i},1),:)];
                isCollision = true;
                checkedTillWaypoint = 1;
                qMatrix = [];
                while (isCollision)
                    startWaypoint = checkedTillWaypoint;
                    for k = startWaypoint:size(qWaypoints,1)-1
                        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(k:k+1,:),deg2rad(10));
                        if ~IsCollision(robot.model,qMatrixJoin,faces,vertex,faceNormals)
                            qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
                            robot.model.animate(qMatrixJoin);
                            size(qMatrix)
                            isCollision = false;
                            checkedTillWaypoint = k+1;
                            % Now try and join to the final goal (q2)
                            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
                            if ~IsCollision(robot.model,qMatrixJoin,faces,vertex,faceNormals)
                                qMatrix = [qMatrix;qMatrixJoin];
                                % Reached goal without collision, so break out
                                break;
                            end
                        else
                            % Randomly pick a pose that is not in collision
                            qRand = (2 * rand(1,3) - 1) * pi;
                            while IsCollision(robot.model,qRand,faces,vertex,faceNormals)
                                qRand = (2 * rand(1,3) - 1) * pi;
                            end
                            qWaypoints =[ qWaypoints(1:k,:); qRand; qWaypoints(k+1:end,:)];
                            isCollision = true;
                            break;
                        end
                    end
                end
                robot.model.animate(qMatrix)
                return
            end
            robot.model.animate(robotTraj{i}(j,:))

            pause(0.01)
            if i == steps * 3.5 % placed tray in dish
                pause(5)
            end
        end
    end
    
end


%% getTrajectory
% Gets the quintic trajectory between two joint states
function robotTraj = getTrajectory(q1, q2)
    steps = 2;
    while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
        steps = steps + 1;
    end
    robotTraj = jtraj(q1,q2,steps);
end


%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
    if nargin < 6
        returnOnceFound = true;
    end
    result = false;
    
    for qIndex = 1:size(qMatrix,1)
        % Get the transform of every joint (i.e. start and end of every link)
        tr = GetLinkPoses(qMatrix(qIndex,:), robot);
    
        % Go through each link and also each triangle face
        for i = 1 : size(tr,3)-1    
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
%                 faceNormals(faceIndex,:);
%                 vertOnPlane;
%                 tr(1:3,4,i)';
%                 tr(1:3,4,i+1)';
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                    plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                    disp('Intersection');
                    result = true;
                    if returnOnceFound
                        return
                    end
                end
            end    
        end
    end
end


%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)

    links = robot.links;
    transforms = zeros(4, 4, length(links) + 1);
    transforms(:,:,1) = robot.base;
    
    for i = 1:length(links)
        L = links(1,i);
        
        current_transform = transforms(:,:, i);
        
        current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
        transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
        transforms(:,:,i + 1) = current_transform;
    end
end


%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

    u = triangleVerts(2,:) - triangleVerts(1,:);
    v = triangleVerts(3,:) - triangleVerts(1,:);
    
    uu = dot(u,u);
    uv = dot(u,v);
    vv = dot(v,v);
    
    w = intersectP - triangleVerts(1,:);
    wu = dot(w,u);
    wv = dot(w,v);
    
    D = uv * uv - uu * vv;
    
    % Get and test parametric coords (s and t)
    s = (uv * wv - vv * wu) / D;
    if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
        result = 0;
        return;
    end
    
    t = (uv * wu - uu * wv) / D;
    if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
        result = 0;
        return;
    end
    
    result = 1;                      % intersectP is in Triangle
end

%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
    if nargin < 3
        maxStepRadians = deg2rad(1);
    end
        
    steps = 2;
    while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
        steps = steps + 1;
    end
    qMatrix = jtraj(q1,q2,steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
    if nargin < 2
        maxStepRadians = deg2rad(1);
    end
    
    qMatrix = [];
    for i = 1: size(waypointRadians,1)-1
        qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
    end
end