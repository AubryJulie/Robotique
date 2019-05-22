function testOfTables(objects)
  %% Initiate the connection to the simulator.

  close all;

  LENGTH = 15;
  RESOLUTION= 128;

  objectNumber = 2;
  firstBool = true;
  nearBasket = false;
  nearObject = false;

  disp('Program started');
  % Use the following line if you had to recompile remoteApi
  %vrep = remApi('remoteApi', 'extApi.h');
  vrep = remApi('remoteApi');
  vrep.simxFinish(-1);
  id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

  % If you get an error like:
  %   Remote API function call returned with error code: 64. Explanation: simxStart was not yet called.
  % Make sure your code is within a function! You cannot call V-REP from a script.

  if id < 0
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
  end
  fprintf('Connection %d to remote API server open.\n', id);

  % Make sure we close the connection whenever the script is interrupted.
  cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

  % This will only work in "continuous remote API server service".
  % See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
  vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

  % Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose.
  h = youbot_init(vrep, id);

  % Let a few cycles pass to make sure there's a value waiting for us next time
  % we try to get a joint angle or the robot pose with the simx_opmode_buffer
  % option.
  pause(.2);

  %% Youbot constants
  timestep = .05;
  LENGTH = 15;
  HEIGHT = 15;
  RESOLUTION= 128;
  gripper = false;
  gripperRelease = false;
  MAXVELOCITY = 1;
  compteur = 0;

  % Definition of the starting pose of the arm.
  startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
  jointsPositions = startingJoints;

  % Parameters for controlling the youBot's wheels.
  forwBackVel = 0;
  leftRightVel = 0;
  rotVel = 0;

  %first fill the map with only 2 which represent unvisited location.
  %should replace this with a macro WALL, UNVISITED and FLOOR
  value_map = ones(RESOLUTION)*2;



  % Make sure everything is settled before we start.
  pause(2);
  [res, ~] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
  vrchk(vrep, res, true);

  % Load the map

  load v.mat;
  value_map = p_map;

  %% Begin execution
  % Set the controller
  holonomicController = HolonomicPursuit([],0.1*MAXVELOCITY,MAXVELOCITY,MAXVELOCITY);

  % ArmController

  armController = ArmController();

  fsm = 'goNearobject';
  armState = 'Nothing';

  % Start the test
  while true
    loop = tic; % See end of loop to see why it's useful.

    if vrep.simxGetConnectionId(id) == -1
      error('Lost connection to remote API.');
    end
    % Get the position and the orientation of the robot.
    % We use the position of the arm as the position of the youbot!
    [res, armPos] = vrep.simxGetObjectPosition(id, h.armRef, -1, vrep.simx_opmode_oneshot);

    vrchk(vrep, res, true);
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    armPosX = armPos(1);
    armPosY = armPos(2);
    youbotTeta = youbotEuler(3);

    if strcmp(fsm,'goNearobject')
      if firstBool
        start = [pts2map(armPosX,RESOLUTION,LENGTH),pts2map(armPosY,RESOLUTION,LENGTH)];
        [goal, angleNearTable] = getPointToPlaceYoubot(armController, objects(objectNumber).center, objects(objectNumber).relyOnTable);
        goal = pts2map(goal, RESOLUTION, LENGTH);
        dx = DXform(value_map,'inflate',3);
        dx.plan(start);
        via = dx.path(round(goal));
        via = via(end:-1:1,:);
        via_r = map2pts(via,RESOLUTION,LENGTH);
        setPathPoints(holonomicController,via_r);
        firstBool = false;
      end


      % Get the robot's speeds
      [forwBackVel, leftRightVel, rotVel] = step(holonomicController,double([armPosX,armPosY,youbotTeta-pi/2]));

      %The position is close enough of the goal. Thus we want to stop.
      if sqrt((armPosX-via_r(end,1))^2+(armPosY-via_r(end,2))^2) < 1
        fsm= 'braking';
        nearObject = true;
      end


    elseif strcmp(fsm,'stay')
      [forwBackVel, leftRightVel, rotVel] = stayNearTable(holonomicController,double([armPosX,armPosY,youbotTeta-pi/2]));

      if abs(forwBackVel)<0.01 && abs(leftRightVel)<0.01 && abs(rotVel)<0.1 && compteur>200
        if nearObject
          nearObject = false;
          setPointToGrabMapFrame(armController, objects(objectNumber).center, double([armPosX,armPosY,youbotTeta-pi/2]));
          armState = 'Pickup';
          compteur = 0;
          firstBool = true;

        elseif nearBasket
          nearBasket = false;
          setPlaceToDrop(armController, objects(objectNumber).basketToDrop, angleToset);
          armState = 'Drop';
          compteur = 0;
          firstBool = true;
        end
      end


    elseif strcmp(fsm,'goToBasket')
      if firstBool
        start = [pts2map(armPosX,RESOLUTION,LENGTH),pts2map(armPosY,RESOLUTION,LENGTH)];
        goal = objects(objectNumber).youbotPosBasketToDrop;
        goal = pts2map(goal, RESOLUTION, LENGTH);
        dx = DXform(value_map,'inflate',3);
        dx.plan(start);
        via = dx.path(round(goal));
        via = via(end:-1:1,:);
        via_r = map2pts(via,RESOLUTION,LENGTH);
        setPathPoints(holonomicController,via_r);
        firstBool = false;
      end


      % Get the robot's speeds
      [forwBackVel, leftRightVel, rotVel] = step(holonomicController,double([armPosX,armPosY,youbotTeta-pi/2]));

      %The position is close enough of the goal. Thus we want to stop.
      if sqrt((armPosX-via_r(end,1))^2+(armPosY-via_r(end,2))^2) < 1
        fsm= 'braking';
        nearBasket = true;
      end


    elseif strcmp(fsm,'braking')
      [forwBackVel, leftRightVel, rotVel, robotStopped] = stopRobot(holonomicController,double([armPosX,armPosY,youbotTeta-pi/2]));

      if robotStopped
        if nearObject
          setTablePosition(holonomicController, objects(objectNumber).relyOnTable);
          setRobotPositionNearTable(holonomicController, 1, angleNearTable);
          fsm = 'stay';
          compteur = 0;
        elseif nearBasket
          [~,angleToset] = getPointToPlaceYoubot(armController, objects(objectNumber).youbotPosBasketToDrop, objects(objectNumber).basketToDrop);
          setTablePosition(holonomicController, objects(objectNumber).basketToDrop);
          setRobotPositionNearTable(holonomicController, 1, angleToset);
          fsm = 'stay';
          compteur = 0;
        end
      end
    end % End if fsm

    if strcmp(armState,'Pickup')
      [jointsPositions, gripper, isDone] = objectPickup(armController, holonomicController, forwBackVel, leftRightVel, rotVel);
      if isDone
        firstBool = true;
        armState = 'Nothing';
        fsm = 'goToBasket';
        compteur = 0;
      end
    elseif strcmp(armState,'Drop')
      [jointsPositions, gripperRelease, isDone] = objectDrop(armController, holonomicController, forwBackVel, leftRightVel, rotVel);
      if isDone
        armState = 'Nothing';
        fsm = 'goNearobject';
        firstBool = true;
        compteur = 0;
        objectNumber = objectNumber + 1;

        if objectNumber > length(objects)
          disp('Done!');
          break;
        end
      end
    end


    % Update wheel velocities using the global values (whatever the state is).
    h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

    time_passed = toc(loop);
    timeleft = timestep - time_passed;
    if timeleft > 0
      pause(min(timeleft,.01));
    end


    % Send joints position
    for i=1:5
      res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), jointsPositions(i), vrep.simx_opmode_oneshot);
      vrchk(vrep, res, true);
    end
    if gripper
      res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);
      pause(2);
    elseif gripperRelease
      res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);
      pause(2);
    end

    compteur = compteur+1;
  end % End of while true

end  % End of function

function [ map ] = pts2map(pts, RESOLUTION, LENGTH)
  map = ceil(pts * RESOLUTION/LENGTH + RESOLUTION/2);
  map(map < 1) = 1;
  map(map > RESOLUTION) = RESOLUTION;
end

function [ pts ] = map2pts(map, RESOLUTION, LENGTH)
  pts = (map - RESOLUTION/2) * LENGTH/RESOLUTION;
end
