classdef main < handle
    properties(Constant)
        startingPos = zeros(1,6);
        robot = motoman(false);
        workspace = [-2,2,-2,2,0,2];
    end
    methods(Static)
        %% Initialise Environment
        function environment() 
            clf
            axis equal
            main.robot.model.animate(main.startingPos)
            
%             main.robot.model.plot(main.startingPos,'workspace',main.workspace,'floorlevel',0,'noarrow', 'scale', 0.8)
%             main.robot.model.teach
            hold on
        end
        
        %% Set path for robot
        function path()
            main.environment();

            pickupDirtyPose = transl(1,1,0);
            washingMachine = transl(1,-1,0);
            dropoffCleanPose = transl(0,-1,1);

            elbowDownDishwasher = [-pi, -pi/8, 3*pi/2, -pi/4, pi/2, 0];
            elbowDownDrier = [-3*pi/2, -pi/8, 3*pi/2, -pi/4, pi/2, 0];

            pickupDirtyQ = main.robot.model.ikcon(pickupDirtyPose, elbowDownDishwasher);
            washingMachineQ = main.robot.model.ikcon(washingMachine, elbowDownDishwasher);
            dropoffCleanQ = main.robot.model.ikcon(dropoffCleanPose, elbowDownDrier);

            steps = 100;
            robotTraj = zeros(steps*4,6);
            
            robotTraj(1:steps,:) = jtraj(main.startingPos, pickupDirtyQ, steps);
            robotTraj(steps+1:steps*2,:) = jtraj(pickupDirtyQ, washingMachineQ, steps);
            robotTraj(steps*2+1:steps*3,:) = jtraj(washingMachineQ, dropoffCleanQ, steps);
            robotTraj(steps*3+1:steps*4,:) = jtraj(dropoffCleanQ, main.startingPos, steps);

            input('go?')

            for i = 1 : steps * 4
                main.robot.model.animate(robotTraj(i,:))
            end

        end
    end
end


% active workspace sensing


% collision detection



% input visual servo


% ability to asynchronously stop the program, as if sommeone had walked
% into the robot environment

% active collision avoidance, simulate object coming in front of robot path


