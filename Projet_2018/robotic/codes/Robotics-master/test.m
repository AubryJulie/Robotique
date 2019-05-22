function test()
  %% Initiate the connection to the simulator.

  close all;

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
  h = youbot_hokuyo_init(vrep, h);

  % Let a few cycles pass to make sure there's a value waiting for us next time
  % we try to get a joint angle or the robot pose with the simx_opmode_buffer
  % option.
  pause(.2);

  %% Youbot constants
  timestep = .05;
  LENGTH = 15.5;
  HEIGHT = 15.5;
  RESOLUTION= 124;
  YOUBOTRADIUS = 0.75;
  MAXVELOCITY = 1;
  MAXROTVEL = pi/2;
  stuckLimit = 300;
  clockLimit = 50;

  % Definition of the starting pose of the arm.
  startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];

  % Parameters for controlling the youBot's wheels.
  forwBackVel = 0;
  leftRightVel = 0;
  rotVel = 0;

  %will be used later when we don't have the information about the
  %position of the robot.
  prevOri = 0;
  prevLoc = 0;
  previousSpeed = 0;
  stuck = 0;
  %first fill the map with only 2 which represent unvisited location.
  %should replace this with a macro WALL, UNVISITED and FLOOR
  step_length = LENGTH/RESOLUTION;
  value_map = ones(RESOLUTION)*2;
  path_map = value_map;
  % Initialise the plot.
  plotData = true;
  plotSomething = true;
  if plotData
    figure('Name','Map');
    ca = gca;
    xlim([1, RESOLUTION]);
    ylim([1, RESOLUTION]);
    xlim manual;
    ylim manual;
    axis equal;
    hold on;
  end

  % Create a 2D mesh of points, stored in the vectors X and Y. This will be used to display the area the robot can
  % see, by selecting the points within this mesh that are within the visibility range.

  [X, Y] = meshgrid(-LENGTH/2:step_length:LENGTH/2, -HEIGHT/2:step_length:HEIGHT/2);
  X = reshape(X, 1, []);
  Y = reshape(Y, 1, []);

  % Make sure everything is settled before we start.
  pause(2);
  via = zeros(1,2);
  via_r = zeros(1,2);
  [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
  vrchk(vrep, res, true);

  %we start by doing a turn about to have a global idea of our position. What ever the first pose is.
  angle = youbotEuler(3) - pi;
  fsm = 'rotate';
  %% Begin execution
  pathController = robotics.PurePursuit('MaxAngularVelocity',MAXROTVEL,'LookaheadDistance', MAXVELOCITY,'DesiredLinearVelocity',MAXVELOCITY);

  % Map to attract the robot to the corners of the map
  dx = Dstar(zeros(RESOLUTION));
  dx.plan([ceil(RESOLUTION/2),ceil(RESOLUTION/2)]);
  distanceMap = dx.distancemap_get();

  while true
    loop = tic; % See end of loop to see why it's useful.

    if vrep.simxGetConnectionId(id) == -1
      error('Lost connection to remote API.');
    end
    % Get the position and the orientation of the robot.
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    youbotPosX = youbotPos(1);
    youbotPosY = youbotPos(2);
    youbotTeta = youbotEuler(3);


    %------------ INTERNAL MAP REPRESNATION ------------%
    transform = se2(youbotPosX,youbotPosY,youbotTeta);
    % Read data from the Hokuyo sensor.
    [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
    pts_homo = [pts(1,:); pts(2,:);ones(1,size(pts,2))];
    % transform point of the Hokuyo into the reference frame

    pts = h2e(transform * pts_homo);

    % Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo.
    in = inpolygon(X, Y, [youbotPosX,pts(1, 1:25:end),youbotPosX],...
    [youbotPosY,pts(2, 1:25:end),youbotPosY]);

    x_map_in = pts2map(X(in),RESOLUTION, LENGTH);
    x_map_on = pts2map(pts(1,contacts),RESOLUTION,LENGTH);
    y_map_in = pts2map(Y(in),RESOLUTION,LENGTH);
    y_map_on = pts2map(pts(2,contacts),RESOLUTION,LENGTH);


    %find all points that are in the polygone;
    indexs_path = sub2ind(size(value_map),y_map_in,x_map_in);
    %find all points in the map which hasn't been set to 1 yet, in fact
    %we don't want to changes ones values. We prefer having more ones
    %representing wall's than 0 representing floor.
    indexs_wall = sub2ind(size(value_map),y_map_on,x_map_on);
    for i=1:length(indexs_path)
      if value_map(indexs_path(i))~=1
        value_map(indexs_path(i)) = 0;
      end
    end
    value_map(indexs_wall) = 1;


    %% Plot something if required.
    if plotData
      if plotSomething
        plot(ca,pts2map(youbotPosX,RESOLUTION,LENGTH), pts2map(youbotPosY,RESOLUTION,LENGTH), 'or');
        hold on;
        imagesc(ca,value_map);
        plot(via(:,1),via(:,2),'*g');
        axis equal;
        hold on;
        drawnow;
        plotSomething = false;
      end
    end


    if strcmp(fsm, 'rotate')

      %% First, rotate the robot to go to one table.
      % The rotation velocity depends on the difference between the current angle and the target.
      rotVel = angdiff(angle, youbotTeta);
      %When the rotation is done (with a sufficiently high precision), move on to the next state.
      if (abs(angdiff(angle, youbotTeta)) < .1 / 180 * pi) && ...
        (abs(angdiff(prevOri, youbotTeta)) < .01 / 180 * pi)
        rotVel = 0;
        plotSomething = true;
        if isempty(pathController.Waypoints)
          fsm = 'explore';
        else
          fsm = 'move';
        end
      end
      prevOri = youbotTeta;

    elseif strcmp(fsm,'explore')

      start = [pts2map(youbotPosX,RESOLUTION,LENGTH),pts2map(youbotPosY,RESOLUTION,LENGTH)];

      [i,j] = find(value_map==2);
      path_map = value_map;
      index= sub2ind(size(path_map),i,j);
      path_map(index) = 0;
      path_map = idilate(path_map, kcircle((pts2map(YOUBOTRADIUS,RESOLUTION,LENGTH) - pts2map(0,RESOLUTION,LENGTH))/2));

      % Make a hole for the robot
      Layers = ceil(YOUBOTRADIUS/2*RESOLUTION/LENGTH);
      for r=start(1)-Layers : start(1)+Layers
        for s = start(2)-Layers : start(2)+Layers
          if r<1
            r=1;
          elseif s<1;
            s=1;
          elseif r>RESOLUTION
            r = RESOLUTION;
          elseif s>RESOLUTION
            s = RESOLUTION;
          end
          path_map(s,r) = 0;
        end
      end

      via = findPahtForExploreV2(value_map, path_map, distanceMap, start, RESOLUTION,LENGTH);
      via_r = map2pts(via,RESOLUTION,LENGTH);
      %%IF -1 -1 there is no new point to explore
      if( via(1,1) == -1 && via(1,2) == -1)
        close all;
        [i,j] = find(value_map==2);
        value_map(sub2ind(size(value_map),i,j)) = 1;
        imagesc(value_map);
        hold on;
        break;
      end

      sindir = asin((via_r(2,2)-via_r(1,2))/(((via_r(2,2)-via_r(1,2))^2+(via_r(2,1)-via_r(1,1))^2)^0.5)); % +pi/2 to match the map frame
      cosdir = acos((via_r(2,1)-via_r(1,1))/(((via_r(2,2)-via_r(1,2))^2+(via_r(2,1)-via_r(1,1))^2)^0.5)); % Different output interval!

      if cosdir>pi/2
        if sindir>0
          % after +pi/2 'third'
          angle = -(sindir+pi/2); % angle in [-pi;pi]
        else
          % after +pi/2 'fourth'
          angle = -(sindir+pi/2);
        end
      else
        if sindir>=0
          % after +pi/2 'second'
          angle = sindir+pi/2;
        else
          % after +pi/2 'first'
          angle = sindir+pi/2;
        end
      end

      pathController.Waypoints = via_r;

      fsm = 'rotate';

      plotSomething = true;

    elseif strcmp(fsm,'move')
      % A wall appeared on the path.
      index = sub2ind(size(value_map),via(:,2),via(:,1));
      if (~isempty(find(value_map(index) == 1,1)))
        clock = 0;
        x = previousSpeed;
        fsm = 'breakdown';
        % Speed at the goal
      elseif sqrt((youbotPosX-via_r(end,1))^2+(youbotPosY-via_r(end,2))^2) < 0.5*MAXVELOCITY
        % We don't care about the linear speed here, as we are close to the end_point of the
        % but we care about the rotation speed. Indeed we still want to follow the curve.
        v = 2*sqrt((youbotPosX-via_r(end,1))^2+(youbotPosY-via_r(end,2))^2);
        [~, w] = step(pathController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));
      else
        [v, w] = step(pathController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));
      end
      %sreduce linear speed in order to keep a trajectory close the the path and
      %try to have a smoth trajectory.
      if abs(w)>0.05
        v = v-v*(abs(w)/MAXROTVEL);
      end
      %the speed is almost null. Thus we want to stop the robot and search for a new path.
      if abs(v) <0.05 && sqrt((youbotPosX-via_r(end,1))^2+(youbotPosY-via_r(end,2))^2) <0.1
        v = 0;
        w = 0;
        pathController.release();
        plotSomething = true;
        fsm= 'explore';
      end
      % The robot is in a static position it could be possible that the robot is stuck
      % against a wall. If this happens for multiple time the robot juste go away
      % from the wall in the state 'stuck'
      if ( abs(prevLoc(1) - youbotPosX) < 0.01 && abs(prevLoc(2) - youbotPosY) < 0.01 && abs(prevOri - youbotTeta) < 0.01)
        stuck = stuck + 1;
      else
        stuck = 0;
      end
      if stuck == 150
        disp('Stuck!');
        % We slow down the wheels in order to go in the other direction.
        x = previousSpeed;
        fsm = 'stuck';
      end

      prevLoc = [youbotPosX youbotPosY];
      prevOri = youbotTeta;
      previousSpeed = -v;

      forwBackVel = -v;
      rotVel = w;

      %for some reason we want to stop

    elseif strcmp(fsm,'breakdown')
      clock = clock+1;
      if x < 0
        w = 0;
        v = x - x*clock/clockLimit;
      else
        w = 0;
        v = x + x*clock/clockLimit;
      end
      if clock >= clockLimit
        v = 0;
        w = 0;
        pathController.release();
        plotSomething = true;
        fsm= 'explore';
      end
      previousSpeed = forwBackVel;
      forwBackVel = v;
      rotVel = w;


      % We are in front of a wall, we go back for some iteration and search
      % for a new path.

    elseif strcmp(fsm,'stuck')
      stuck = stuck + 1;


      if x>=0
        v = -0.6+0.6*stuck/stuckLimit;
        w = 0;
      else
        v = 0.6-0.6*stuck/stuckLimit;
        w = 0;
      end


      if stuck >=stuckLimit;
        v = 0;
        w = 0;
        pathController.release();
        plotSomething = true;
        fsm= 'explore';
      end
      previousSpeed = forwBackVel;
      forwBackVel  = v;
      rotVel = w;

    end

    % Update wheel velocities using the global values (whatever the state is).
    h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

    time_passed = toc(loop);
    timeleft = timestep - time_passed;
    if timeleft > 0
      pause(min(timeleft,.01));
    end
  end
end

function [via] = findPahtForExploreV2(value_map, path_map, distanceMap,start, RESOLUTION,LENGTH)
  ds = Dstar(path_map);
  ds.plan(start);
  cost_map = ds.distancemap_get();
  percentage = 0.05;

  for r=ceil(percentage*RESOLUTION):RESOLUTION-ceil(percentage*RESOLUTION)
    for s=ceil(percentage*RESOLUTION):RESOLUTION-ceil(percentage*RESOLUTION)
      cost_map(r,s) = cost_map(r,s)-distanceMap(r,s);
    end
  end

  [i,j] = find(path_map==1);
  value_map(sub2ind(size(value_map),i,j)) = 1;
  [i,j] = find(value_map==2);
  distances = cost_map(sub2ind(size(cost_map),i,j));

  potentialTargets = [i,j,distances];

  % Sort for closest 2's first
  potentialTargets = sortrows(potentialTargets, 3);

  for r=ceil(percentage*RESOLUTION):RESOLUTION-ceil(percentage*RESOLUTION)
    for s=ceil(percentage*RESOLUTION):RESOLUTION-ceil(percentage*RESOLUTION)
      if cost_map(r,s)<1
        cost_map(r,s) = 1; % To avoid strange behaviors due to small costs
      end
    end
  end

  goal = [potentialTargets(1,2),potentialTargets(1,1)];

  via = ds.path(goal);
  via = flipud(via);

  index = sub2ind(size(cost_map),via(:,2),via(:,1));
  if ~isempty(find(value_map(index)==1,1))
    via = [-1 -1];
    return;
  end
  % there is a wall on the path, we cannot explore futher.
end


function [ map ] = pts2map(pts, RESOLUTION, LENGTH)
  map = ceil(pts * RESOLUTION/LENGTH + RESOLUTION/2);
end

function [ pts ] = map2pts(map, RESOLUTION, LENGTH)
  pts = (map - RESOLUTION/2) * LENGTH/RESOLUTION;
end
