function test()
  %% Initiate the connection to the simulator.

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
  RESOLUTION = 256;
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

  %first fill the map with only 2 which represent unvisited location.
  %should replace this with a macro WALL, UNVISITED and FLOOR
  step = LENGTH/RESOLUTION;
  value_map = ones(RESOLUTION)*2;

  % Initialise the plot.
  plotData = true;
  if plotData
    figure('Name','Map');
    ca = gca;
    xlim([1, RESOLUTION]);
    ylim([1, RESOLUTION]);
    xlim manual;
    ylim manual;
    hold on;

    % Create a 2D mesh of points, stored in the vectors X and Y. This will be used to display the area the robot can
    % see, by selecting the points within this mesh that are within the visibility range.

    [X, Y] = meshgrid(-LENGTH/2:step:LENGTH/2, -HEIGHT/2:step:HEIGHT/2);
    X = reshape(X, 1, []);
    Y = reshape(Y, 1, []);
  end

  % Make sure everything is settled before we start.
  pause(2);

  [res, homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
  vrchk(vrep, res, true);
  fsm = 'rotate';

  processor = 'record';
  index = 1;
  %% Begin execution
  while true
    tic % See end of loop to see why it's useful.

    if vrep.simxGetConnectionId(id) == -1
      error('Lost connection to remote API.');
    end

    % Get the position and the orientation of the robot.
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);


    % Read data from the Hokuyo sensor.
    if strcmp(processor,'record')
      [pts(:,:,index), contacts(:,:,index)] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
      youbotPosX(index) = youbotPos(1);
      youbotPosY(index) = youbotPos(2);
      youbotTeta(index) = youbotEuler(3);

      transform(:,:,index) = se2(youbotPosX(index),youbotPosY(index),youbotTeta(index));

    elseif strcmp(processor,'process')
      for i = 1:30:index-1
        pts_homo = [pts(1,:, i); pts(2,:, i);ones(1,size(pts,2))];
        pts2 = h2e(transform(:,:,i) * pts_homo);


        % Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo.
        in = inpolygon(X, Y, [youbotPosX(i),pts2(1, :),youbotPosX(i)],...
        [youbotPosY(i),pts2(2, :),youbotPosY(i)]);

        % Plot those points. Green dots: the visible area for the Hokuyo. Red starts: the obstacles. Red lines: the
        % visibility range from the Hokuyo sensor.
        % The youBot is indicated with two dots: the blue one corresponds to the rear, the red one to the Hokuyo
        % sensor position.
        x_map_in = ceil(X(in) * RESOLUTION/LENGTH + RESOLUTION/2);
        x_map_on = ceil(pts2(1,contacts(:,:,i)) * RESOLUTION/LENGTH + RESOLUTION/2);
        y_map_in = ceil(Y(in) * RESOLUTION/LENGTH + RESOLUTION/2);
        y_map_on = ceil(pts2(2,contacts(:,:,i)) * RESOLUTION/LENGTH + RESOLUTION/2);
        indexs_path = sub2ind(size(value_map),y_map_in,x_map_in);
        indexs_wall = sub2ind(size(value_map),y_map_on,x_map_on);
        for j=1:2:length(indexs_path)-1
          disp(j)
          indexs_path(j)
          if value_map(indexs_path(j),indexs_path(j+1))~=1
            value_map(indexs_path(j),indexs_path(j+1)) = 0;
          end
        end
        value_map(indexs_wall) = 1;
      end
      if plotData
        processor = 'plot';
      else
        processor = 'end';
      end
    elseif strcmp(processor,'plot')
      pcolor(ca,value_map);
      processor = 'end';
    end

    angle = pi;
    if strcmp(fsm, 'rotate')
      %% First, rotate the robot to go to one table.
      % The rotation velocity depends on the difference between the current angle and the target.
      rotVel = angdiff(angle, youbotEuler(3));
      index = index+1;
      % When the rotation is done (with a sufficiently high precision), move on to the next state.
      if (abs(angdiff(angle, youbotEuler(3))) < .1 / 180 * pi) && ...
        (abs(angdiff(prevOri, youbotEuler(3))) < .01 / 180 * pi)
        rotVel = 0;

        if strcmp(processor,'record')
          processor = 'process';
        end

        if strcmp(processor,'end')
          fsm = 'drive';
          processor = 'record';
        end
      end
      prevOri = youbotEuler(3);
    elseif strcmp(fsm, 'drive')
      %% Then, make it move straight ahead until it reaches the table.
      % The further the robot, the faster it drives. (Only check for the first dimension.)
      forwBackVel = -(youbotPos(1) + 3);

      % If the robot is sufficiently close and its speed is sufficiently low, stop it and move its arm to
      % a specific location before moving on to the next state.
      if (youbotPos(1) + 3 < .001) && (abs(youbotPos(1) - prevLoc) < .001)
        forwBackVel = 0;

        % Change the orientation of the camera
        vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0 0 pi/4], vrep.simx_opmode_oneshot);
        % Move the arm to the preset pose.
        for i = 1:5
          res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), pickupJoints(i),...
          vrep.simx_opmode_oneshot);
          vrchk(vrep, res, true);
        end
        break;
      end
      prevLoc = youbotPos(1);
    end

    % Update wheel velocities using the global values (whatever the state is).
    h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

    time_passed = toc;
    timeleft = timestep - time_passed;
    if timeleft > 0
      pause(min(timeleft,.01));
    end
  end

end
