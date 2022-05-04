function [] = main2()
    clf

    startingPos = zeros(1,6);
    robot = motoman(false);
    workspace = [-2,2,-2,2,0,2];
    
    
    axis equal
    robot.model.animate(zeros(1,6));
%     robot.model.teach
%     input('yo')
    
%     robot.model.plot(startingPos,'workspace',workspace,'floorlevel',0,'noarrow', 'scale', 0.8)
%     robot.model.teach
%     robot.model.delay = 0;
    hold on
%     input('a')

    pickupDirtyPose = transl(1,1,0.2) * troty(pi/2);
    washingMachine = transl(-1,0,0.2) * troty(-pi/2);
    dropoffCleanPose = transl(1,-1,0.5)*troty(pi/2);

    elbowDownDishwasher = [pi, pi/4, -pi/2, pi/4, -pi/2, 0];
    elbowDownDrier = [0, pi/4, -pi/2, pi/4, -pi/2, 0];
    
    pickupDirtyWaypointQ = robot.model.ikcon(pickupDirtyPose * transl(0,0,-0.3),elbowDownDrier);
    pickupDirtyQ = robot.model.ikcon(pickupDirtyPose, elbowDownDrier);
    washingMachineWaypointQ = robot.model.ikcon(washingMachine * transl(0,0,-0.3), elbowDownDishwasher);
    washingMachineQ = robot.model.ikcon(washingMachine, elbowDownDishwasher);
    dropoffCleanWaypointQ = robot.model.ikcon(dropoffCleanPose * transl(0,0,-0.3), elbowDownDrier);
    dropoffCleanQ = robot.model.ikcon(dropoffCleanPose, elbowDownDrier);

    steps = 150;
    robotTraj = zeros(steps*7,6);
    
    robotTraj(1:steps*0.5,:) = jtraj(startingPos, pickupDirtyWaypointQ, steps/2);
    robotTraj(steps*0.5+1:steps*1.5,:) = jtraj(pickupDirtyWaypointQ, pickupDirtyQ, steps);
    robotTraj(steps*1.5+1:steps*2,:) = jtraj(pickupDirtyQ, pickupDirtyWaypointQ, steps/2);
    robotTraj(steps*2+1:steps*2.5,:) = jtraj(pickupDirtyWaypointQ, washingMachineWaypointQ, steps/2);
    robotTraj(steps*2.5+1:steps*3.5,:) = jtraj(washingMachineWaypointQ, washingMachineQ, steps);
    robotTraj(steps*3.5+1:steps*4,:) = jtraj(washingMachineQ, washingMachineWaypointQ, steps/2);
    robotTraj(steps*4+1:steps*4.5,:) = jtraj(washingMachineWaypointQ, dropoffCleanWaypointQ, steps/2);
    robotTraj(steps*4.5+1:steps*5.5,:) = jtraj(dropoffCleanWaypointQ, dropoffCleanQ, steps);
    robotTraj(steps*5.5+1:steps*6,:) = jtraj(dropoffCleanQ, dropoffCleanWaypointQ, steps/2);
    robotTraj(steps*6+1:steps*7,:) = jtraj(dropoffCleanWaypointQ, startingPos, steps);

    for i = 1 : steps * 7
        robot.model.animate(robotTraj(i,:))
        pause(0.01)
        if i == steps * 3.5 % placed tray in dish
            pause(5)
        end
    end
end