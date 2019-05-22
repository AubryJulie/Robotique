function [objects,pts_world] = explore2()
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
  initialRadius = 1;
  gripper = false;
  gripperRelease = false;
  MAXVELOCITY = 1;
  
  
  % Definition of the starting pose of the arm.
  startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];

  % Parameters for controlling the youBot's wheels.
  forwBackVel = 0;
  leftRightVel = 0;
  rotVel = 0;

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
  %will be used later when we don't have the information about the
  %position of the robot.
  prevOri = 0;
  prevLoc = 0;
  stuck = 0;
  isStopped = false;
  %first fill the map with only 2 which represent unvisited location.
  %should replace this with a macro WALL, UNVISITED and FLOOR
  step_length = LENGTH/RESOLUTION;
  value_map = ones(RESOLUTION)*2;
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
  % Make sure everything is settled before we start.
  pause(2);
  via = zeros(1,2);
  via_r = zeros(1,2);
  [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
  vrchk(vrep, res, true);
  [~, INIT_POS] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
  INIT_POS = pts2map(INIT_POS,RESOLUTION,LENGTH);
  %we start by doing a turn about to have a global idea of our position. What ever the first pose is.
  angle1 = youbotEuler(3) - pi;


  %% Begin execution
  % Set the controller
  holonomicController = HolonomicPursuit([],0.1*MAXVELOCITY,MAXVELOCITY,MAXVELOCITY);
  
  
  
  %%TODO change this !!!!!!!!!!!
  load('map2.mat');
  
  [baskets,tables,r,value_map]=locate(value_map, INIT_POS);
  n_table = 1;
  n_basket = 0;
  armController = ArmController();
  
  fsm = 'place';
  disp('place');
  armState = 'Nothing';
  
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
    
    
    %% Plot something if required.
    if plotData
      if plotSomething
        plot(ca,pts2map(youbotPosX,RESOLUTION,LENGTH), pts2map(youbotPosY,RESOLUTION,LENGTH), 'or');
        hold on;
        imagesc(ca,value_map);
        plot(via(:,1),via(:,2),'*g');
        plot(via(1,1),via(1,2),'*r');
        plot(via(end,1),via(end,2),'*k');
        axis equal;
        hold on;
        drawnow;
        plotSomething = false;
      end
    end
 
    if strcmp(fsm,'place')
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
        %move robot near basket
        else
            if(n_basket <= length(baskets))
                start = [pts2map(youbotPosX,RESOLUTION,LENGTH),pts2map(youbotPosY,RESOLUTION,LENGTH)];
                goal = baskets(n_basket).path;
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
        end
        
    elseif strcmp(fsm,'stay')
        res = vrep.simxSetIntegerSignal(h.id, 'handle_xy_sensor', 0, vrep.simx_opmode_oneshot);
        [forwBackVel, leftRightVel, rotVel] = stayNearTable(holonomicController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));
        
        if abs(forwBackVel) < 0.005 && abs(leftRightVel) < 0.005 && rotVel< 0.01
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
        if(precise_table_center)
            pts = pts(1:3, pts(4, :) < 1);
        else
            pts = pts(1:3, pts(4, :) < 1);
        end
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
        close all;
        pcshow(pts_world');
        hold on;
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
                
                disp('grapObject');
                fsm = 'grapObject';
                setNewRadiusNearTable(holonomicController,0.4+0.3);
                %%end of detection
                break;
                n_obj = 1;
            end
        
        end
        
    elseif strcmp(fsm,'grapObject')    
        [forwBackVel, leftRightVel, rotVel] = stayNearTable(holonomicController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));
        
        if abs(forwBackVel) < 0.005 && abs(leftRightVel) < 0.005 && rotVel< 0.01
           disp('alignToObject');
           fsm = 'alignToObject';
        end
        
        
    elseif strcmp(fsm,'alignToObject')
        
        opp = (objects(n_obj).center(2)-tables(n_table).realCenter(2));
        adj = (objects(n_obj).center(1)-tables(n_table).realCenter(1));
        hyp = sqrt(opp^2+adj^2);
        ang = atan(opp./adj);
        if((adj/hyp)<0)
            ang = ang + pi;
        end
        setNewAngleNearTable(holonomicController, ang);
        disp('alignArm')
        fsm = 'alignArm';
        
    elseif strcmp(fsm,'alignArm')
        [res, armPos] = vrep.simxGetObjectPosition(id, h.armRef, -1, vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        [forwBackVel, leftRightVel, rotVel, isStopped] = changeAngleNearTable(holonomicController,double([armPos(1),armPos(2),youbotTeta-pi/2]));
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
        objects(n_obj).color = objectColor(image(round(resolution/2)-5:1:round(resolution/2)+5,round(resolution/2)-5:1:round(resolution/2)+5,:));
        objects(n_obj).basket = findBasket(objects(n_obj),inst);
        if(n_obj >= 5)
            break;
        else
            disp('alignToObject');
            fsm = 'alignToObject';
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
        fsm = 'braking';
      else
        % Normal case
        [forwBackVel,leftRightVel, rotVel] = step(holonomicController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));

      end

      %The position is close enough of the goal. Thus we want to stop.
      if sqrt((youbotPosX-via_r(end,1))^2+(youbotPosY-via_r(end,2))^2) < 0.1
        fsm= 'braking';
      end
      prevLoc = [youbotPosX youbotPosY];
      prevOri = youbotTeta;

      %for some reason we want to stop
      
      
      
      elseif strcmp(fsm,'braking')

      [forwBackVel, leftRightVel, rotVel, robotStopped] = stopRobot(holonomicController,double([youbotPosX,youbotPosY,youbotTeta-pi/2]));

      if robotStopped
        if(n_table <= length(tables))
            pts_world = [];
            setTablePosition(holonomicController,map2pts(tables(n_table).center,RESOLUTION,LENGTH));
            % Set the position of the robot near that table
            setRobotPositionNearTable(holonomicController,map2pts(r,RESOLUTION,LENGTH)-map2pts(0,RESOLUTION,LENGTH)+0.5,depth_table_ang(n_depth));
        else
            setTablePosition(holonomicController,map2pts(baskets(n_basket).center,RESOLUTION,LENGTH));
            % Set the position of the robot near that table
            opp = (baskets(n_basket).path(2)-baskets(n_basket).center(2));
            adj = (baskets(n_basket).path(1)-baskets(n_basket).center(1));
            hyp = sqrt(opp^2+adj^2);
            ang = atan(opp./adj);
            if((adj/hyp)<0)
                ang = ang + pi;
            end
            setRobotPositionNearTable(holonomicController,map2pts(r,RESOLUTION,LENGTH)-map2pts(0,RESOLUTION,LENGTH)+0.3,ang);
        end
        disp('stay');
        fsm = 'stay';
      end
      
      
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

function [ map ] = pts2map(pts, RESOLUTION, LENGTH)
  map = round(pts * RESOLUTION/LENGTH + RESOLUTION/2);
  map(map < 1) = 1;
  map(map > RESOLUTION) = RESOLUTION;
end

function [ pts ] = map2pts(map, RESOLUTION, LENGTH)
  pts = (map - RESOLUTION/2) * LENGTH/RESOLUTION;
end
