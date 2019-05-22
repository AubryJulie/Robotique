function explore()
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
  LENGTH = 15;
  HEIGHT = 15;
  RESOLUTION= 128;
  YOUBOTRADIUS = 0.65;
  MAXVELOCITY = 1;
  compteur = 0;

  % Definition of the starting pose of the arm.
  startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
  jointsPositions = startingJoints;

  % Parameters for controlling the youBot's wheels.
  forwBackVel = 0;
  leftRightVel = 0;
  rotVel = 0;

  %will be used later when we don't have the informations about the
  %position of the robot.
  prevOri = 0;
  prevLoc = 0;
  stuck = 0;

  pic = 1;
  load instructions.mat;
  %number of picture per position for detecting the table
  t_pics = 1;
  %number of picture per postition for detecting objects;
  o_pics = 3;
  o_pic_factor = [1 0 -1];
  pics = t_pics;


  n_depth = 1;
  depth_table = 8;
  depth_table_ang = linspace(pi/2,5*pi/2,depth_table);
  depth_object = 16;
  depth_object_ang = linspace(pi/2,5*pi/2,depth_object);

  precise_table_center = true;
  %first fill the map with only 2 which represent unvisited location.
  %should replace this with a macro WALL, UNVISITED and FLOOR
  step_length = LENGTH/RESOLUTION;
  value_map = ones(RESOLUTION)*2;
  path_map = value_map;

  % Initialise the plot.
  plotData = true;
  plotSomething = true;
  detect = false;

  if plotData
    figure('Name','Youbot');
    caTop = subplot(3,2,1:4);
    caBL = subplot(3,2,5);
    caBR = subplot(3,2,6);

    caTop.DataAspectRatio = [1 1 1];
    caTop.PlotBoxAspectRatio = [3 4 4];


    caTop.XLimMode = 'manual';
    caTop.YLimMode = 'manual';
    caTop.ZLimMode = 'manual';
    caTop.XLim = [1, RESOLUTION];
    caTop.YLim = [1, RESOLUTION];

    caBL.XTick = [];
    caBL.YTick = [];

    caBR.XTick = [];
    caBR.YTick = [];

    caTop.XTick = [];
    caTop.YTick = [];

    hold(caTop,'on');

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
  % we get the inital position of the youbot.
  [~, INIT_POS] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
  INIT_POS = pts2map(INIT_POS,RESOLUTION,LENGTH);
  %we start by doing a turn about to have a global idea of our position. What ever the first pose is.
  angle = youbotEuler(3) - pi;
  fsm = 'rotating';

  %% Begin execution
  % Set the controller
  holonomicController = HolonomicPursuit([],0.1*MAXVELOCITY,MAXVELOCITY,MAXVELOCITY);

  % Start with a rotaion
  setDesiredOrientation(holonomicController,angle);

  % Map to attract the robot to the corners of the map
  dx = Dstar(zeros(RESOLUTION));
  dx.plan([ceil(RESOLUTION/2),ceil(RESOLUTION/2)]);
  distanceMap = dx.distancemap_get();

  %variables for state changing.
  explore = true;
  landmark = false;
  detect = false;
  hokuyoboolean = true;
  current_path = [];
  table = [];

  %got to grasping objects.
  transitionFlag = false;



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

    if hokuyoboolean
      % ------------ INTERNAL MAP REPRESNATION ------------ %
      transform = se2(youbotPosX,youbotPosY,youbotTeta);

      if( explore == true && detect == false)
        % Read data from the Hokuyo sensor.
        [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
        pts_homo = [pts(1,:); pts(2,:);ones(1,size(pts,2))];
        % transform point of the Hokuyo into the reference frame

        pts = h2e(transform * pts_homo);

        % Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo.
        in = inpolygon(X, Y, [youbotPosX,pts(1, 1:30:end),youbotPosX],...
        [youbotPosY,pts(2, 1:30:end),youbotPosY]);

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
      end
    end

    %% Plot something if required.
    if plotData
      subplot(caTop);
      if plotSomething
        plot(pts2map(youbotPosX,RESOLUTION,LENGTH), pts2map(youbotPosY,RESOLUTION,LENGTH), 'or');
        imagesc(caTop,value_map);
        if detect
          viscircles(basket_c,3*ones(length(basket_c),1),'color','g');
          for i = 1 : length(baskets)
            current_path_plot = baskets(i).path;
            plot(caTop,current_path_plot(:,1),current_path_plot(:,2),'*r');
            if ~isempty(current_path)
              plot(current_path(:,1),current_path(:,2),'*k');
            end
          end
          for i = 1 : length(table_c)
            viscircles(table_c,3*ones(length(table_c),1),'color','r');
          end
        end
        plot(via(:,1),via(:,2),'*g');
        drawnow;
        plotSomething = false;
      end
    end

    %% -----------Final Sate Machine------------ %%

    if strcmp(fsm, 'rotating')

      [forwBackVel, leftRightVel, rotVel, robotStopped] = rotate(holonomicController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));

      %When the rotation is done (with a sufficiently high precision), moving on to the next state.
      if robotStopped
        plotSomething = true;
        if isPathEmpty(holonomicController)
          fsm = 'exploring';
        else
          fsm = 'moving';
        end
      end

    elseif strcmp(fsm,'exploring')
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

      via = findPahtForExploreV2(value_map, path_map, distanceMap, start, RESOLUTION);
      via_r = map2pts(via,RESOLUTION,LENGTH);

      % If path is too short = unknown points just behind the robot
      if size(via_r,1) < 4
        fsm = 'rotating';
        setDesiredOrientation(holonomicController,youbotTeta+pi);
        setPathPoints(holonomicController,[]);

        %%IF -1 -1 there is no new point to explore
        if( via(1,1) == -1 && via(1,2) == -1)
          explore = false;
          [i,j] = find(value_map==2);
          value_map(sub2ind(size(value_map),i,j)) = 1;
          %End of exploration. Now we detect the tables and the landmarks.
          fsm = 'detect';
        end
      else

        %set the controller

        setPathPoints(holonomicController,via_r);
        %next state
        fsm = 'moving';

        plotSomething = true;
      end % End if size

    elseif strcmp(fsm,'detect')
      %shutting down the hokuyo
      res = vrep.simxSetIntegerSignal(h.id, 'handle_xy_sensor', 0, vrep.simx_opmode_oneshot);
      hokuyoboolean = false;
      %function detecting the tables.
      [baskets,tables,radius,path_map_c,p_map]=locate(value_map, INIT_POS);
      basket_c = [baskets(:).center];
      basket_c = reshape(basket_c,[2 length(baskets)])';
      table_c = [tables(:).center];
      table_c = reshape(table_c,[2 length(tables)])';
      explored_basket = [basket_c zeros(length(baskets),1) (1:length(baskets))'];
      value_map = path_map_c;
      b_number = 1;
      detect = true;
      landmark = true;
      disp('landmark');
      fsm = 'landmark';

    elseif strcmp(fsm,'landmark')
      if(b_number>length(baskets))
        % use of a bag of feature to recognize the landmarks.
        disp('BoF');
        fsm = 'BoF';

      else
        start = [pts2map(youbotPosX,RESOLUTION,LENGTH),pts2map(youbotPosY,RESOLUTION,LENGTH)];
        dx = DXform(path_map_c,'inflate',3);
        dx.plan(start);
        to_explore = explored_basket(explored_basket(:,3)==0,:);
        vias = cell(size(to_explore,1),1);
        for i = 1 : size(to_explore,1)
          %first point we want to reach for a given basket
          current_path = baskets(to_explore(i,4)).path;
          first_goal = round(current_path(1,:));
          goal = [first_goal(1,1) first_goal(1,2)];
          %path to this point
          vias{i} = dx.path(goal);
          %length of the path
          to_explore(i,3) = length(vias{i});
        end
        %shortest path's index
        [~,index] = min(to_explore(:,3));
        index_basket = to_explore(index,4);
        %mark this basket as explored
        explored_basket(index_basket,3) = b_number;
        %get the path
        current_path = baskets(index_basket).path;
        %get the basket center
        current_basket = baskets(index_basket).center;
        current_object = baskets(index_basket).object;
        via = vias{index};
        %reverse Via because we used DXform in a revers way.
        via = via(end:-1:1,:);
        via_r = map2pts(via,RESOLUTION,LENGTH);
        setPathPoints(holonomicController,via_r);
        disp('moving');
        fsm = 'moving';
        plotSomething = true;
      end

    elseif strcmp(fsm,'snapshot')
      % orient the camera with the given angle relatively to the global
      % space
      vrep.simxSetObjectOrientation(id, h.rgbdCasing, -1,[0 0 ang], vrep.simx_opmode_oneshot);

      res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/4, vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);
      % start the sensor, take a pitcure and imediately shut down the
      % sensor.
      res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);
      [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);
      % cut of almost the half of the bottom of the image which is
      % essantialy ground. Du to the orientation of the camera.
      image = image(1:round(end/1.7),1:round(4*end/5),:);

      if plotData
        subplot(caBL);
        imshow(image);
      end

      % create a new directory to store the pictures.
      if(exist('imgs')~=7)
        mkdir('imgs');
      end
      cd 'imgs';
      str1 = 'A';
      str1(1)=int2str(b_number);
      if(exist(str1)~=7)
        mkdir(str1);
      end
      cd(str1);
      string = 'testB.png';
      string(5)=int2str(pic);
      imwrite(image,string);
      cd ../..;
      fsm = 'centerEstimate';


    elseif strcmp(fsm,'centerEstimate')

      [res, cameraPos] = vrep.simxGetObjectPosition(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res, true);
      cameraX = cameraPos(1);
      cameraY = cameraPos(2);

      [res, cameraEuler] = vrep.simxGetObjectOrientation(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res, true);
      q = Quaternion(eul2tr(cameraEuler));

      res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/6, vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);

      res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);

      pts_c = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
      %get points of the wall

      pts_c = pts_c(1:3, pts_c(4,:) < norm([youbotPosX youbotPosY] - [map2pts(current_basket(1),RESOLUTION,LENGTH) map2pts(current_basket(2),RESOLUTION,LENGTH)]));
      pts_c = [pts_c(3,:);pts_c(1,:);pts_c(2,:)];
      for i = 1 : size(pts_c,2)
        pts_c(:,i) = q*double(pts_c(:,i))+cameraPos';
      end
      pts_c = pts_c(1:3, pts_c(3,:) < 0.17);
      pts_c = pts_c(1:3, pts_c(3,:) > 0.01);
      ptCloudb = pointCloud(pts_c');
      center = [0 0];
      n = 0;
      for i = 1 : 5
        maxDistance = 0.005;
        referenceVector = [0,0,1];
        [model,~] = pcfitcylinder(ptCloudb,maxDistance,referenceVector,'confidence',99.99999);
        if(~isempty(model) && model.Radius < 0.42 && model.Radius > 0.38)
          center = (center + model.Center(1:2));
          n = n + 1;
        end
      end
      center = center/n;
      if ~isnan(center)
        baskets(index_basket).realCenter = center;
      else
        baskets(index_basket).realCenter = map2pts(baskets(index_basket).center,RESOLUTION,LENGTH);
      end
      %we have one basket less to explore (redundancy with marked basket)
      b_number=b_number+1;
      %find the closest basket.
      if pic < size(current_path,1)
        subplot(caTop);
        via = round([current_path(pic,:); current_path(pic+1,:)]);
        plot(via(:,1),via(:,2),'y');
        via_r = map2pts(via,RESOLUTION,LENGTH);
        setPathPoints(holonomicController,via_r);
        changeMaxVelocity(holonomicController,0.3,0.3);
        disp('moving to next position');
        fsm='moving';
        pic = pic+1;
      else
        changeMaxVelocity(holonomicController,MAXVELOCITY,MAXVELOCITY);
        disp('landmark');
        fsm='landmark';
        pic = 1;
      end

    elseif strcmp(fsm,'BoF')
      % Set the training folder which are the image captured by the robot
      trainFolder = fullfile('imgs');
      train = imageSet(trainFolder,'recursive');

      % Get a bag of feature from those images
      bag = bagOfFeatures(train,'verbose',false);
      %train a classifier
      categoryClassifier = trainImageCategoryClassifier(train,bag,'verbose',false);
      % get all the reference pictures
      list = dir('pictures/*.png');
      cd pictures;
      score_mat = zeros(length(list));
      for i=1 : length(list)
        img = imread(fullfile(list(i).name));
        % classify all the images
        [~, score] = predict(categoryClassifier,img);
        score_mat(i,:) = score;
      end
      cd ..;
      % get all possible permutation of image(i) = object(j)
      permutation = perms(1:5);
      indexing = 0:5:20;
      best_score = - Inf;
      score_mat = score_mat.';
      for i = 1 : size(permutation,1)
        % for each permutation compute the best score.
        score = sum(score_mat(permutation(i,:) + indexing));
        if( score > best_score )
          best_score = score;
          result = permutation(i,:);
        end
      end
      basket_travel = explored_basket(:,3);
      for i = 1 : length(result)
        baskets(i).picture = strcat('pictures/',list(result==basket_travel(i)).name);
      end

      disp('place');
      fsm = 'place';
      landmark = false;
      n_table = 1;

      %place the robot next to a table.
    elseif strcmp(fsm,'place')
      if(n_table <= length(tables))
        start = [pts2map(youbotPosX,RESOLUTION,LENGTH),pts2map(youbotPosY,RESOLUTION,LENGTH)];
        goal = tables(n_table).path;
        dx = DXform(value_map,'inflate',3);
        dx.plan(start);
        via = dx.path(round(goal));
        via = via(end:-1:1,:);
        via_r = map2pts(via,RESOLUTION,LENGTH);
        setPathPoints(holonomicController,via_r);
        disp('moving');
        fsm = 'moving';
        plotSomething = true;
      end

      %stick the robot close to a table
    elseif strcmp(fsm,'stay')
      [forwBackVel, leftRightVel, rotVel] = stayNearTable(holonomicController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));

      if abs(forwBackVel) < 0.005 && abs(leftRightVel) < 0.005 && rotVel< 0.01 && compteur > 300
        if transitionFlag
          break; % Move objects
        end
        disp('table_pos');
        fsm = 'table_pos';
        n_pic = 1;
      end

    elseif strcmp(fsm,'table_pos')
      %Get Camera pos
      [res, cameraPos] = vrep.simxGetObjectPosition(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res, true);
      cameraX = cameraPos(1);
      cameraY = cameraPos(2);
      %center the camera to the center of the table.
      if(precise_table_center)
        opp = (cameraY-map2pts(tables(n_table).center(2),RESOLUTION,LENGTH));
        adj = (cameraX-map2pts(tables(n_table).center(1),RESOLUTION,LENGTH));
      else
        opp = (cameraY-tables(n_table).realCenter(2));
        adj = (cameraX-tables(n_table).realCenter(1));
      end
      hyp = sqrt(opp^2+adj^2);
      ang = atan(opp./adj);
      if((adj/hyp)<0)
        ang = ang + pi;
      end
      ang = ang + pi;

      if(precise_table_center)
        vrep.simxSetObjectOrientation(id, h.rgbdCasing, -1,[0 0 ang], vrep.simx_opmode_oneshot);
      else
        %take multiple point clouds at a given position
        vrep.simxSetObjectOrientation(id, h.rgbdCasing, -1,[0 0 ang-(o_pic_factor(n_pic)*pi/8)], vrep.simx_opmode_oneshot);
      end

      %get the pose of the camera
      [res, cameraEuler] = vrep.simxGetObjectOrientation(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res, true);
      q = Quaternion(eul2tr(cameraEuler));

      % Reduce the view angle to better see the objects. Indeed, the number of rays the Hokuyo sends is limited;
      % if this number is used on a smaller angle, then the resolution is better.
      if(precise_table_center)
        res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/4, vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res);
      else
        res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/8, vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res);
      end
      % Ask the sensor to turn itself on, take A SINGLE 3D IMAGE, and turn itself off again.
      res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);

      pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
      n_pic = n_pic + 1;
      % Each column of pts has [x;y;z;distancetosensor]. However, plot3 does not have the same frame of reference as
      % the output data. To get a correct plot, you should invert the y and z dimensions.
      % Here, we only keep points within 1 meter, to focus on the table.
      pts = pts(1:3, pts(4, :) < 1);
      pts = [pts(3,:);pts(1,:);pts(2,:)];
      for i = 1 : size(pts,2)
        pts_w(:,i) = q*double(pts(:,i))+cameraPos';
      end

      %taking pts belonging to the tables
      if(precise_table_center)
        pts_w = pts_w(1:3,pts_w(3,:) < 0.17);
        pts_w = pts_w(1:3,pts_w(3,:) > 0.01);
        depth = depth_table;
        pics = t_pics;
      else
        %We have a correct estimation of the table
        pts_w = pts_w(1:3,pts_w(3,:) > 0.1855);
        depth = depth_object;
        pics = o_pics;
      end
      pts_world = horzcat(pts_world,pts_w);

      if plotData
        subplot(caBR);
        pcshow(pts_world');
        axis equal;
      end
      if(n_depth < depth && n_pic > pics)
        n_depth = n_depth + 1;
        if(precise_table_center == true)
          setNewAngleNearTable(holonomicController, depth_table_ang(n_depth));
          n_pic = 1;
        else
          setNewAngleNearTable(holonomicController, depth_object_ang(n_depth));
          n_pic = 1;
        end
        fsm ='changeAngle';
      elseif (n_depth >= depth)
        %find a precise location for the table
        ptCloud = pointCloud(pts_world');
        if(precise_table_center == true)
          center = [0 0];
          rad = 0;
          n = 0;
          for i = 1 : 5
            maxDistance = 0.00005;
            referenceVector = [0,0,1];
            [model,inlierIndices] = pcfitcylinder(ptCloud,maxDistance,referenceVector,'confidence',0.99999);
            if(model.Radius < 0.42 && model.Radius > 0.38)
              center = (center + model.Center(1:2));
              rad = (rad + model.Radius);
              n = n + 1;
            end
          end
          center = center/n;
          rad = rad/n;
          tables(n_table).realCenter = center;
          tables(n_table).realRadius = rad;

          n_depth = 1;
          pts_world = [];
          precise_table_center = false;
          setTablePosition(holonomicController,tables(n_table).realCenter);
          % Set the position of the robot near that table
          setRobotPositionNearTable(holonomicController,0.4+0.22,depth_table_ang(n_depth));
          computer = 0;
          isStopped = false;
          disp('stay');
          fsm = 'stay';
        else

          %% Detect objects
          objects = detecObjects(pts_world);
          disp('objectColor');
          fsm = 'objectColor';
          %%end of detection
          n_obj = 1;
        end

      end



      %should be renamed with detect color.
    elseif strcmp(fsm,'objectColor')
      [forwBackVel, leftRightVel, rotVel] = stayNearTable(holonomicController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));

      if abs(forwBackVel) < 0.005 && abs(leftRightVel) < 0.005 && rotVel< 0.01
        disp('angToObject');
        fsm = 'angToObject';
      end

      %find angle with the object and the center of the table

    elseif strcmp(fsm,'angToObject')

      opp = (objects(n_obj).center(2)-tables(n_table).realCenter(2));
      adj = (objects(n_obj).center(1)-tables(n_table).realCenter(1));
      hyp = sqrt(opp^2+adj^2);
      ang = atan(opp./adj);
      if((adj/hyp)<0)
        ang = ang + pi;
      end
      setNewAngleNearTable(holonomicController, ang);
      disp('alignRobot')
      fsm = 'alignRobot';

    elseif strcmp(fsm,'alignRobot')
      [forwBackVel, leftRightVel, rotVel, isStopped] = changeAngleNearTable(holonomicController,double([youbotPosX, youbotPosY,youbotTeta-pi/2]));
      if isStopped
        disp('colorPic');
        fsm = 'colorPic';
      end

    elseif strcmp(fsm,'colorPic')
      [res, cameraPos] = vrep.simxGetObjectPosition(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res, true);
      cameraX = cameraPos(1);
      cameraY = cameraPos(2);
      %center the camera to the center of the table.
      opp = (cameraY-objects(n_obj).center(2));
      adj = (cameraX-objects(n_obj).center(1));
      hyp = sqrt(opp^2+adj^2);
      ang = atan(opp./adj);
      if((adj/hyp)<0)
        ang = ang + pi;
      end
      ang = ang + pi;
      vrep.simxSetObjectOrientation(id, h.rgbdCasing, -1,[0 0 ang], vrep.simx_opmode_oneshot);
      % start the sensor, take a pitcure and imediately shut down the
      % sensor.
      res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);
      [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);



      objects(n_obj).color = objectColor(image(round(resolution(1)/2)-5:1:round(resolution(1)/2)+5,round(resolution(2)/2)-5:1:round(resolution(2)/2)+5,:));
      objects(n_obj).basket = findBasket(objects(n_obj),inst);

      objects(n_obj).relyOnTable = tables(n_table).realCenter;
      for i = 1 : length(baskets)
        if(strcmp(objects(n_obj).basket,baskets(i).picture))
          objects(n_obj).basketToDrop = baskets(i).realCenter;
          objects(n_obj).youbotPosBasketToDrop = map2pts(baskets(i).path,RESOLUTION,LENGTH);
        end

      end

      if(n_obj >= 5)
        %setNewRadiusNearTable(holonomicController,1); % Get in a safe position for the following setTablePosition
        transitionFlag = true;
        compteur = 0;
        fsm = 'stay';
      else
        disp('angToObject');
        fsm = 'angToObject';
        n_obj = n_obj + 1;
      end

    elseif strcmp(fsm,'changeAngle')
      [forwBackVel, leftRightVel, rotVel, isStopped] = changeAngleNearTable(holonomicController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));
      if isStopped
        fsm = 'table_pos';
      end


    elseif strcmp(fsm,'moving')
      % A wall appeared on the path.
      index = sub2ind(size(value_map),via(:,2),via(:,1));
      if (~isempty(find(value_map(index) == 1,1)))
        disp('breaking');
        fsm = 'braking';
      else
        % Normal case
        [forwBackVel,leftRightVel, rotVel] = step(holonomicController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));

      end

      %The position is close enough of the goal. Thus we want to stop.
      if sqrt((youbotPosX-via_r(end,1))^2+(youbotPosY-via_r(end,2))^2) < 1
        disp('breaking');
        fsm= 'braking';
      end

      % The robot is in a static position it could be possible that the robot is stuck
      % against a wall. If this happens for multiple time the robot juste go away
      % from the wall in the state 'stuck'
      if ( abs(prevLoc(1) - youbotPosX) < 0.01 && abs(prevLoc(2) - youbotPosY) < 0.01 && abs(prevOri - youbotTeta) < 0.01)
        stuck = stuck + 1;
      else
        stuck = 0;
      end
      if stuck >= 150
        disp('Stuck!');
        stuck = 0;
        fsm = 'stuck';
      end

      prevLoc = [youbotPosX youbotPosY];
      prevOri = youbotTeta;

      %for some reason we want to stop

    elseif strcmp(fsm,'braking')

      [forwBackVel, leftRightVel, rotVel, robotStopped] = stopRobot(holonomicController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));

      if robotStopped
        if (explore == true && landmark == false)
          plotSomething = true;
          disp('exploring');
          fsm= 'exploring';
        elseif ( explore == false && landmark == true)
          setTablePosition(holonomicController,map2pts(current_basket,RESOLUTION,LENGTH));
          % Set the position of the robot near that table
          opp = (pts2map(youbotPosY,RESOLUTION,LENGTH)-current_object(2));
          adj = (pts2map(youbotPosX,RESOLUTION,LENGTH)-current_object(1));
          hyp = sqrt(opp^2+adj^2);
          ang = atan(opp./adj);
          %opposit direction of the computed angle
          setPathPoints(holonomicController,[]);

          if(pic==1)
            if((adj/hyp)<0)
              ang = ang + pi;
            end
            ang = ang - pi/2;
            setDesiredOrientation(holonomicController,ang);
            disp('align');
            fsm = 'align';
          else
            if((adj/hyp)<0)
              ang = ang + pi;
            end
            ang = ang + pi;
            disp('snapshot')
            fsm = 'snapshot';
          end
        elseif (explore == false && landmark == false)
          pts_world = [];
          setTablePosition(holonomicController,map2pts(tables(n_table).center,RESOLUTION,LENGTH));
          % Set the position of the robot near that table
          setRobotPositionNearTable(holonomicController,map2pts(radius,RESOLUTION,LENGTH)-map2pts(0,RESOLUTION,LENGTH)+0.5,depth_table_ang(n_depth));
          disp('stay');
          fsm = 'stay';
        end
      end

      % State to change pos of the robot near a table. We don't want the
      % robot to appear on the picture.
    elseif strcmp(fsm,'align')

      [forwBackVel, leftRightVel, rotVel, robotStopped] = rotate(holonomicController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));

      %When the rotation is done (with a sufficiently high precision), moving on to the next state.
      %Orient the camera if necessary.
      if robotStopped
        plotSomething = true;

        opp = (pts2map(youbotPosY,RESOLUTION,LENGTH)-current_object(2));
        adj = (pts2map(youbotPosX,RESOLUTION,LENGTH)-current_object(1));
        hyp = sqrt(opp^2+adj^2);
        ang = atan(opp./adj);
        if((adj/hyp)<0)
          ang = ang + pi;
        end
        ang = ang + pi;

        disp('snapshot');
        fsm = 'snapshot';
      end

      % We are in front of a wall, we go back for some iteration and search
      % for a new path.

    elseif strcmp(fsm,'stuck')

      [forwBackVel, leftRightVel, rotVel, robotStopped] = destuck(holonomicController);

      if robotStopped
        plotSomething = true;
        fsm= 'exploring';
      end

    end

    % Update wheel velocities using the global values (whatever the state is).
    h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

    time_passed = toc(loop);
    timeleft = timestep - time_passed;
    if timeleft > 0
      pause(min(timeleft,.01));
    end
    compteur = compteur +1;
  end % End of the first loop


  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  close all;
  % Transport of the differents objects part

  % Variables to run this part
  objectNumber = 1;

  firstBool = true;
  nearBasket = false;
  nearObject = false;
  gripper = false;
  gripperRelease = false;
  compteur = 0;

  % ArmController

  armController = ArmController();

  fsm = 'goNearobject';
  armState = 'Nothing';


  % Second loop to move the objects
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

        path_map = idilate(path_map_c, kcircle(3));

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
        dx = DXform(path_map);
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
        path_map = idilate(path_map_c, kcircle(3));

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
        dx = DXform(path_map);
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
      if sqrt((armPosX-via_r(end,1))^2+(armPosY-via_r(end,2))^2) < 0.3
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

    prevLocX = youbotPosX;
    prevLocY = youbotPosY;
    prevTeta = youbotTeta;

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

end % End of function explore


function [via] = findPahtForExploreV2(value_map, path_map, distanceMap,start, RESOLUTION)
  ds = Dstar(path_map);
  ds.plan(start);
  cost_map = ds.distancemap_get();
  percentage = 0.05;
  for r=ceil(percentage*RESOLUTION):RESOLUTION-ceil(percentage*RESOLUTION)
    for s=ceil(percentage*RESOLUTION):RESOLUTION-ceil(percentage*RESOLUTION)
      cost_map(r,s) = cost_map(r,s)-distanceMap(r,s);
    end
  end
  cost_map(cost_map<0) = 1;
  %figure;
  %imagesc(cost_map);
  ds.distancemap_set(cost_map);
  ds.plan();
  [i,j] = find(path_map==1);
  value_map(sub2ind(size(value_map),i,j)) = 1;
  [i,j] = find(value_map==2);
  distances = cost_map(sub2ind(size(cost_map),i,j));

  potentialTargets = [i,j,distances];

  % Sort for closest 2's first
  potentialTargets = sortrows(potentialTargets, 3);

  goal = [potentialTargets(1,2),potentialTargets(1,1)];
  via = ds.path(goal);
  via = flipud(via);
  if size(via,1) == 0 && size(via,2) == 0
    via = [-2 -2];
    return;
  end
  index = sub2ind(size(cost_map),via(:,2),via(:,1));
  if ~isempty(find(value_map(index)==1,1))
    via = [-1 -1];
    return;
  end
  % there is a wall on the path, we cannot explore futher.
end


function [ map ] = pts2map(pts, RESOLUTION, LENGTH)
  map = ceil(pts * RESOLUTION/LENGTH + RESOLUTION/2);
  map(map < 1) = 1;
  map(map > RESOLUTION) = RESOLUTION;
end

function [ pts ] = map2pts(map, RESOLUTION, LENGTH)
  pts = (map - RESOLUTION/2) * LENGTH/RESOLUTION;
end
