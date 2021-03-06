% Demonstration of the specified task during an online (in class time) group 
% presentation (10-15 minutes per group): Path plan between several poses and 
% the final joint state given a unique and safe environment (developed by each group) 
% and the simulated model. Use RMRC, collision avoidance and a GUI where appropriate. 
% Creatively use a real robot that mimics and/or enhances the simulation and application. 
% Depending upon the robot, marks may be scaled to ignore the hareware portion, 
% but only if the hardware is inaccessible due to university-approved reasons (e.g. travel restrictions).

% Safety in Demo:
% (1) xSystem reacts to userxs emergency stop action (minus marks if no estop) 
% (2) Trajectory reacts to simulated sensor input (e.g. light curtain)
% (3) Trajectory reacts to a forced simulated upcoming collision
% (4) Make the robot retreat from a simulated safety symbol using visual servoing and RMRC

% RMRC - Lab 9


function [] = main()
    clf

    %% Environment Setup
    hold on;
    tablePosition = [-0.75,-1.15,0; 0,0,0; 0.8,-1.15,0];
    tableMesh_h = PlaceObject('newTable.ply', tablePosition);

%     ; -1.5,0.5,0; -1.5,-0.5,0

    tableVertices = get(tableMesh_h(2),'Vertices');
    tableTransformedVertices = [tableVertices,ones(size(tableVertices,1),1)] * trotz(pi/2)';
    set(tableMesh_h(2),'Vertices',tableTransformedVertices(:,1:3));

    tableMesh_h(4) = PlaceObject('newTable.ply');
    tablePose = transl(-1.2,1,0) * trotz(pi/2);
    tableVertices = get(tableMesh_h(4),'Vertices');
    tableTransformedVertices = [tableVertices,ones(size(tableVertices,1),1)] * tablePose';
    set(tableMesh_h(4),'Vertices',tableTransformedVertices(:,1:3));

    tableMesh_h(5) = PlaceObject('newTable.ply');
    tablePose = transl(-1.2,-0.2,0) * trotz(pi/2);
    tableVertices = get(tableMesh_h(5),'Vertices');
    tableTransformedVertices = [tableVertices,ones(size(tableVertices,1),1)] * tablePose';
    set(tableMesh_h(5),'Vertices',tableTransformedVertices(:,1:3));

    
%     tablePose = transl(tablePosition) * trotz(pi);
    
%     tableVertices = get(tableMesh_h,'Vertices');
%     tableTransformedVertices = [tableVertices,ones(size(tableVertices,1),1)] * tablePose';
%     set(tableMesh_h,'Vertices',tableTransformedVertices(:,1:3));
    
    axis equal
    
    hold on;
    camlight;
    
    basePosition = [0 1.7 0];
    basePose = transl(basePosition);
    baseMesh_h = PlaceObject('dishwasherBottom_d.ply');
    baseVertices = get(baseMesh_h,'Vertices');
    baseTransformedVertices = [baseVertices,ones(size(baseVertices,1),1)] * basePose';
    set(baseMesh_h,'Vertices',baseTransformedVertices(:,1:3));
    
    rackPosition = [0.9 -1.2 1];
    rackPose = transl(rackPosition);
    rackMesh_h = PlaceObject('dishTray.ply');
    rackVertices = get(rackMesh_h,'Vertices');
    rackTransformedVertices = [rackVertices,ones(size(rackVertices,1),1)] * rackPose';
    set(rackMesh_h,'Vertices',rackTransformedVertices(:,1:3));
    
    lidPosition = [0 1.5 1.7];
    lidPose = transl(lidPosition);
    lidMesh_h = PlaceObject('toplid_d.ply');
    lidVertices = get(lidMesh_h,'Vertices');
    lidTransformedVertices = [lidVertices,ones(size(lidVertices,1),1)] * lidPose';
    set(lidMesh_h,'Vertices',lidTransformedVertices(:,1:3));

    firstAidPosition = [2.38 0 1.1];
    firstAidPose = transl(firstAidPosition) * trotz(-pi/2);
    firstAidMesh_h = PlaceObject('firstAid.ply');
    firstAidVertices = get(firstAidMesh_h,'Vertices');
    firstAidTransformedVertices = [firstAidVertices,ones(size(firstAidVertices,1),1)] * firstAidPose';
    set(firstAidMesh_h,'Vertices',firstAidTransformedVertices(:,1:3));

    fireExtPosition = [2.4 0 0];
    fireExtPose = transl(fireExtPosition)  * trotz(pi/2) ;
    fireExtMesh_h = PlaceObject('fireExtinguisher.ply');
    fireExtVertices = get(fireExtMesh_h,'Vertices');
    fireExtTransformedVertices = [fireExtVertices,ones(size(fireExtVertices,1),1)] * fireExtPose';
    set(fireExtMesh_h,'Vertices',fireExtTransformedVertices(:,1:3));

    eStopPosition = [0.35 -1.4 0.95];
    eStopPose = transl(eStopPosition);
    eStopMesh_h = PlaceObject('eStop.ply');
    eStopVertices = get(eStopMesh_h,'Vertices');
    eStopTransformedVertices = [eStopVertices,ones(size(eStopVertices,1),1)] * eStopPose';
    set(eStopMesh_h,'Vertices',eStopTransformedVertices(:,1:3));

    fencePosition = [0 2.2 0];
    fencePose = transl(fencePosition);
    fenceMesh_h = PlaceObject('fence.ply');
    fenceVertices = get(fenceMesh_h,'Vertices');
    fenceTransformedVertices = [fenceVertices,ones(size(fenceVertices,1),1)] * fencePose';
    set(fenceMesh_h,'Vertices',fenceTransformedVertices(:,1:3));
    
    fence2Position = [-2 0 0];
    fence2Pose = transl(fence2Position)  * trotz(pi/2) ;
    fence2Mesh_h = PlaceObject('fence.ply');
    fence2Vertices = get(fence2Mesh_h,'Vertices');
    fence2TransformedVertices = [fence2Vertices,ones(size(fence2Vertices,1),1)] * fence2Pose';
    set(fence2Mesh_h,'Vertices',fence2TransformedVertices(:,1:3));
    
    % Surface environment setup
    surf([2.5,2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[2.5,2.5;0,0],'CData',imread('kitchen2.jpg'),'FaceColor','texturemap');
    surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');


    %% Logic
    startingPos = zeros(1,6);
    robot = motoman(false);
    workspace = [-2,2,-2,2,0,2];

    robot.model.base = robot.model.base * transl(0,0,0.9);
    robot.model.animate(startingPos);
    xlim([-2.5 2.5]) 
    ylim([-2.5 2.5])
    zlim([0 2.5])
    
%     robot.model.plot(startingPos,'workspace',workspace,'floorlevel',0,'noarrow', 'scale', 0.4)
%     robot.model.teach
    input('pause')
    
%     centerpnt = [0.75,0.7,2];
%     centerpnt = [0.75,0.7,1.7];
    centerpnt = [0,0.7,1.7];
    side = 0.5;
    plotOptions.plotFaces = true;
    [objVertex,objFaces,objFaceNormals,objHandle] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

    % Main poses to pick/dropoff tray
    pickupDirtyPose = transl(0.9, -0.9, 1.05) * trotx(pi/2);
    washingMachine = transl(0,1.55,1.15) * troty(pi/2)*trotx(-pi/2)*trotz(pi/2);
    dropoffCleanPose = transl(-1,-.9,1.02)* trotx(pi/2);

    % Estimate joint angles
    elbowDownDishwasher = [pi/2, -pi/4, pi/4, -pi/2, 0, 0];
    elbowDownDrier = [-pi/2, -pi/4, pi/4, -pi/2, 0, 0];
    
    % Setting robot joint angle waypoints to move through
    pickupDirtyWaypointQ = robot.model.ikcon(pickupDirtyPose * transl(0,0.3,-0.3),elbowDownDrier);
    pickupDirtyQ = robot.model.ikcon(pickupDirtyPose, elbowDownDrier);
    washingMachineWaypointQ = robot.model.ikcon(washingMachine * transl(0,0.2,-0.7), elbowDownDishwasher);
    washingMachineQ = robot.model.ikcon(washingMachine, elbowDownDishwasher);
    dropoffCleanWaypointQ = robot.model.ikcon(dropoffCleanPose * transl(0,0.3,-0.3), elbowDownDrier);
    dropoffCleanQ = robot.model.ikcon(dropoffCleanPose, elbowDownDrier);

    %% Building trajectories
    % Normal Speed
    normalTraj{1} = getTrajectory(startingPos, pickupDirtyWaypointQ,4,robot);
%     normalTraj{2} = getTrajectory(pickupDirtyWaypointQ, pickupDirtyQ,3,robot);
    normalTraj{2} = getRMRCTrajectory(pickupDirtyWaypointQ, [0.9 -0.9 1.05 pi/2 0 pi/2],3,robot);
%     normalTraj{3} = getTrajectory(normalTraj{2}(size(normalTraj{2},1),:), pickupDirtyQ,1.5, robot);
    normalTraj{3} = getTrajectory(pickupDirtyQ, pickupDirtyWaypointQ,3,robot);
    normalTraj{4} = getTrajectory(pickupDirtyWaypointQ, washingMachineWaypointQ,5,robot);
    normalTraj{5} = getTrajectory(washingMachineWaypointQ, washingMachineQ,3,robot);
    normalTraj{6} = getTrajectory(washingMachineQ, washingMachineWaypointQ,3,robot);
%     normalTraj{7} = getTrajectory(washingMachineWaypointQ, washingMachineQ,3,robot);
    normalTraj{7} = getRMRCTrajectory(washingMachineWaypointQ, [0 1.55 1.15 -pi/2 0 -pi],3,robot);
%     normalTraj{9} = getTrajectory(normalTraj{8}(size(normalTraj{8},1),:), washingMachineQ,1.5, robot);
    normalTraj{8} = getTrajectory(washingMachineQ, washingMachineWaypointQ,3,robot);
    normalTraj{9} = getTrajectory(washingMachineWaypointQ, dropoffCleanWaypointQ,5,robot);
    normalTraj{10} = getTrajectory(dropoffCleanWaypointQ, dropoffCleanQ,3,robot);
    normalTraj{11} = getTrajectory(dropoffCleanQ, dropoffCleanWaypointQ,3,robot);
    normalTraj{12} = getTrajectory(dropoffCleanWaypointQ, startingPos,4,robot);

    % Slow Speed
    slowTraj{1} = getTrajectory(startingPos, pickupDirtyWaypointQ,8,robot);
%     slowTraj{2} = getTrajectory(pickupDirtyWaypointQ, pickupDirtyQ,6,robot);
    slowTraj{2} = getRMRCTrajectory(pickupDirtyWaypointQ, [0.9 -0.9 1.05 pi/2 0 pi/2],6,robot);
%     slowTraj{3} = getTrajectory(slowTraj{2}(size(slowTraj{2},1),:), pickupDirtyQ,1.5, robot);
    slowTraj{3} = getTrajectory(pickupDirtyQ, pickupDirtyWaypointQ,6,robot);
    slowTraj{4} = getTrajectory(pickupDirtyWaypointQ, washingMachineWaypointQ,10,robot);
    slowTraj{5} = getTrajectory(washingMachineWaypointQ, washingMachineQ,6,robot);
    slowTraj{6} = getTrajectory(washingMachineQ, washingMachineWaypointQ,6,robot);
%     slowTraj{7} = getTrajectory(washingMachineWaypointQ, washingMachineQ,6,robot);
    slowTraj{7} = getRMRCTrajectory(washingMachineWaypointQ, [0 1.55 1.15 -pi/2 0 -pi],6,robot);
%     slowTraj{9} = getTrajectory(slowTraj{8}(size(slowTraj{8},1),:), washingMachineQ,1.5, robot);
    slowTraj{8} = getTrajectory(washingMachineQ, washingMachineWaypointQ,6,robot);
    slowTraj{9} = getTrajectory(washingMachineWaypointQ, dropoffCleanWaypointQ,10,robot);
    slowTraj{10} = getTrajectory(dropoffCleanWaypointQ, dropoffCleanQ,6,robot);
    slowTraj{11} = getTrajectory(dropoffCleanQ, dropoffCleanWaypointQ,6,robot);
    slowTraj{12} = getTrajectory(dropoffCleanWaypointQ, startingPos,8,robot);

    prevRackLocation = rackPose * trotx(pi/2) * transl(0,0.05,-0.3);

    % Animation and logic
    stop = false;
    slow = false;
    count = 0;
    robotTraj{1} = slowTraj{1};

    for i = 1 : 12
%         if i == 3
%             input('check')
%         end
        success = false;
        
        steps = size(slowTraj{i});
        for x = 1 : size(slowTraj{i},1)
%             if i == 6 && x == 5
%                 slow = true;
%             elseif i == 9 && x == 100
%                 slow = false;
%             end
            %% Collision check for 100 steps infront
            j = x;
            if rem(x,2) ~= 0 && ~slow
                continue
            elseif ~slow
                j = x / 2;
            end

            if stop
                while(stop)
                end
            end
            
            if slow
                robotTraj{i} = slowTraj{i};
                if i ~= 12
                    robotTraj{i+1} = slowTraj{i+1};
                end
            else
                robotTraj{i} = normalTraj{i};
                if i ~= 12
                    robotTraj{i+1} = normalTraj{i+1};
                end
            end
            
            % Every 20 steps, check the next 200 steps
            count = count + 1;
            if count > 19
                for k = 0 : 199
                    count2 = 1;
                    if (j+k > size(robotTraj{i},1)) % needs to check current movement and next
                        % current iteration is outside movement, so check
                        % next movement
                        if (i ~= 12 && IsCollision(robot.model,robotTraj{i+1}(count2,:),objFaces,objVertex,objFaceNormals,false))
                            disp('Collision in path')
                            [success, prevRackLocation] = avoidance(robot, robotTraj{i+1}, count2,objFaces,objVertex,objFaceNormals,i,rackMesh_h, prevRackLocation);
                            if success
                                break;
                            else
                                while(IsCollision(robot.model,robotTraj{i+1}(count2,:),objFaces,objVertex,objFaceNormals,false))
                                    pause(2.5);
                                    delete(objHandle)
                                    objFaces = [];
                                    objVertex = [];
                                    objFaceNormals = [];
                                    drawnow
                                end
                            end
                            
                        end
                        count2 = count2 + 1;

                    else % there is more than 200 steps left in current movement
                        if (IsCollision(robot.model,robotTraj{i}(j+k,:),objFaces,objVertex,objFaceNormals,false))
                            disp('Collision in path')
                            [success, prevRackLocation] = avoidance(robot, robotTraj{i}, j+k,objFaces,objVertex,objFaceNormals,i,rackMesh_h, prevRackLocation);
                            if success
                                break;
                            else
                                while(IsCollision(robot.model,robotTraj{i}(j+k,:),objFaces,objVertex,objFaceNormals,false))
                                    pause(2.5);
                                    delete(objHandle)
                                    objFaces = [];
                                    objVertex = [];
                                    objFaceNormals = [];
                                    drawnow
                                end
                            end
                        end 
                    end
                end
                if success
                    break;
                end
                count = 0;
            end
            
            if i == 3 || i == 4 || i == 5 || i == 8 || i == 9 || i == 10
                tr = (robot.model.fkine(robotTraj{i}(j,:)) / prevRackLocation);
                vertices = get(rackMesh_h,'Vertices');
                transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
                set(rackMesh_h,'Vertices',transformedVertices(:,1:3));
                drawnow();
                prevRackLocation = robot.model.fkine(robotTraj{i}(j,:));
%                 pause(0.2)
            end

            if i == 7 && x == 2
                for k = 1:20
                    tr = transl(0,0,0.03);
                    vertices = get(lidMesh_h,'Vertices');
                    transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
                    set(lidMesh_h,'Vertices',transformedVertices(:,1:3));
                    drawnow();
                end
            end

            robot.model.animate(robotTraj{i}(j,:))
            pause(0.01)

            if i == 6 && x == steps(1)
                for k = 1:20
                    tr = transl(0,0,-0.03);
                    vertices = get(lidMesh_h,'Vertices');
                    transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
                    set(lidMesh_h,'Vertices',transformedVertices(:,1:3));
                    drawnow();
                end
                pause(5)
            end
        end
    end
    
end

%% getTrajectory
% Gets the quintic trajectory between two joint states
function qMatrix = getRMRCTrajectory(q1, goal, time, robot)
    t = time;             % Total time (s)
    deltaT = 0.02;      % Control frequency
    steps = t/deltaT;   % No. of steps for simulation
    delta = 2*pi/steps; % Small angle change
    epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
    
    % 1.2) Allocate array data
    m = zeros(steps,1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps,6);       % Array for joint anglesR
    qdot = zeros(steps,6);          % Array for joint velocities
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    x = zeros(3,steps);             % Array for x-y-z trajectory
    positionError = zeros(3,steps); % For plotting trajectory error
    angleError = zeros(3,steps);    % For plotting trajectory error

    x1 = robot.model.fkine(q1);
    x1 = x1(1:3,4);
    x2 = goal(1:3)';

    x = zeros(3,steps);
    s = lspb(0,1,steps);                                 % Create interpolation scalar
    for i = 1:steps
        x(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y plane
        theta(1,i) = goal(4);                 % Roll angle 
        theta(2,i) = goal(5);            % Pitch angle
        theta(3,i) = goal(6);
    end
     
    T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    q0 = zeros(1,6);                                                            % Initial guess for joint angles
    qMatrix(1,:) = q1;                                            % Solve joint angles to achieve first waypoint
    
    % 1.4) Track the trajectory with RMRC
    for i = 1:steps-1
        T = robot.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
        deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
        S = Rdot*Ra';                                                           % Skew symmetric!
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
        deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
        xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
        J = robot.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
        qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
        for j = 1:6                                                             % Loop through joints 1 to 6
            if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
        positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
        angleError(:,i) = deltaTheta;                                           % For plotting
    end

end


function qMatrix = getTrajectory(q1, q2, time, robot)
    t = time;             % Total time (s)
    deltaT = 0.02;      % Control frequency
    steps = t/deltaT;

    qMatrix = jtraj(q1,q2,steps);
end


%% Collision Avoidance
function [success, prevRackLocation] = avoidance(robot, robotTraj, count2,objFaces,objVertex,objFaceNormals,i,rackMesh_h, prevRackLocation)
    % get robot to move in other direction
    currentPos = robot.model.getpos;
    qFinal = currentPos;
    % get direction of rotation
    rotation = robotTraj(count2,1) - robotTraj(count2 - 1,1);
    if rotation > 0
        qFinal(1) = qFinal(1) - deg2rad(170);
        robotTraj(size(robotTraj,1),1) = robotTraj(size(robotTraj,1),1) - deg2rad(360);
    else
        qFinal(1) = qFinal(1) + deg2rad(170);
        robotTraj(size(robotTraj,1),1) = robotTraj(size(robotTraj,1),1) + deg2rad(360);
    end
    
    % Create new trajectories along new path
    newTraj1 = jtraj(currentPos, qFinal, 300);
    newTraj2 = jtraj(qFinal, robotTraj(size(robotTraj,1),:), 300);

    % Check for collisions along new path
    collision = false;
    for l = 1 : 300
        if (IsCollision(robot.model,newTraj1(l,:),objFaces,objVertex,objFaceNormals,false))
            collision = true;
        end
        if (IsCollision(robot.model,newTraj2(l,:),objFaces,objVertex,objFaceNormals,false))
            collision = true;
        end
    end

    if collision
        disp('Cannot reach goal without collision, please remove object to continue.')
        success = false;
    else
        for l = 1 : 300
            if i == 3 || i == 4 || i == 5 || i == 8 || i == 9 || i == 10
                tr = (robot.model.fkine(newTraj1(l,:)) / prevRackLocation);
                vertices = get(rackMesh_h,'Vertices');
                transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
                set(rackMesh_h,'Vertices',transformedVertices(:,1:3));
                drawnow();
                prevRackLocation = robot.model.fkine(newTraj1(l,:));
            end

            robot.model.animate(newTraj1(l,:));
            pause(0.01);
        end
        for l = 1 : 300
            if i == 3 || i == 4 || i == 5 || i == 8 || i == 9 || i == 10
                tr = (robot.model.fkine(newTraj2(l,:)) / prevRackLocation);
                vertices = get(rackMesh_h,'Vertices');
                transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
                set(rackMesh_h,'Vertices',transformedVertices(:,1:3));
                drawnow();
                prevRackLocation = robot.model.fkine(newTraj2(l,:));
            end

            robot.model.animate(newTraj2(l,:));
            pause(0.01);
        end
        success = true;
    end
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
%                     disp('Intersection');
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




    %             if (IsCollision(robot.model,robotTraj{i}(j,:),faces,vertex,faceNormals,false))
    %                 disp("COLLISION")
    % 
    %                 qWaypoints = [robotTraj{i}(j,:); robotTraj{i}(size(robotTraj{i},1),:)];
    %                 isCollision = true;
    %                 checkedTillWaypoint = 1;
    %                 qMatrix = [];
    %                 while (isCollision)
    %                     startWaypoint = checkedTillWaypoint;
    %                     for k = startWaypoint:size(qWaypoints,1)-1
    %                         qMatrixJoin = InterpolateWaypointRadians(qWaypoints(k:k+1,:),deg2rad(10));
    %                         if ~IsCollision(robot.model,qMatrixJoin,faces,vertex,faceNormals)
    %                             qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
    %                             robot.model.animate(qMatrixJoin);
    %                             size(qMatrix)
    %                             isCollision = false;
    %                             checkedTillWaypoint = k+1;
    %                             % Now try and join to the final goal (q2)
    %                             qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
    %                             if ~IsCollision(robot.model,qMatrixJoin,faces,vertex,faceNormals)
    %                                 qMatrix = [qMatrix;qMatrixJoin];
    %                                 % Reached goal without collision, so break out
    %                                 break;
    %                             end
    %                         else
    %                             % Randomly pick a pose that is not in collision
    %                             qRand = (2 * rand(1,6) - 1) * pi;
    %                             while IsCollision(robot.model,qRand,faces,vertex,faceNormals)
    %                                 qRand = (2 * rand(1,6) - 1) * pi;
    %                             end
    %                             qWaypoints =[ qWaypoints(1:k,:); qRand; qWaypoints(k+1:end,:)];
    %                             isCollision = true;
    %                             break;
    %                         end
    %                     end
    %                 end
    %                 robot.model.animate(qMatrix)
    %                 return
    %             end