function youbotfinalendtest2()
    %% Initiate the connection to the simulator.    
    disp('Program started');
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);

    % Make sure we close the connection whenever the script is interrupted.
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % This will only work in "continuous remote API server service". 
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose. 
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);
    pause(.2);

    %% Youbot constants.
    % The time step the simulator is using (your code should run close to it). 
    timestep = .05;

    % Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
    startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
       
    %% Initial values.
    disp('Starting robot');

    forwBackVel = 0; % Move straight ahead. 
    rightVel = 0; % Go sideways. 
    rotateRightVel = 0; % Rotate. 
    prevOrientation = 0; % Previous angle to goal (easy way to have a condition on the robot's angular speed). 
    prevPosition = 0; % Previous distance to goal (easy way to have a condition on the robot's speed). 

    % Set the arm to its starting configuration. 
    res = vrep.simxPauseCommunication(id, true); % Send order to the simulator through vrep object. 
    vrchk(vrep, res); % Check the return value from the previous V-REP call (res) and exit in case of error.
    
    % Set the target position of a joint.
    for i = 1:5
        res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
    end    
    res = vrep.simxPauseCommunication(id, false); 
    vrchk(vrep, res);
    
   % Get the position and the orientation of the robot. 
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);


    % fullcode = true -> run completely the code
    % fullcode = false -> run sequentially
    fullcode = false;
    interfullcode = fullcode;
    
    % Initialise save matrix for navigation. 
    initTraj = true;
    minMap = [0 0];
    maxMap = [0 0];
    minMapInter = [0 0];
    maxMapInter = [0 0];
    NewGoal = [0 0];
    resol = 4; % 25 cm.

    % Initialise save matrix for navigation. 
    initTraj = true;
    minMap = [0 0];
    maxMap = [0 0];
    minMapInter = [0 0];
    maxMapInter = [0 0];
    NewGoal = [0 0];
    resol = 4; % 25 cm.
    PositionUpdateMap = [youbotPos(1) youbotPos(2)];
    AngleUpdateMap = youbotEuler(3);
    
    counter = 1;
    counterspeed = 0;
    
    % MaxDist is the distance after which we reach the maximum speed.
    maxSpeed = 30;
    maxDist = 40;
    

    % Initial square map of lenght 12m.
    totalMap = ones(12*resol+1);
    
    % Grid to store the points from the area that the hokuyo captor sees.
    distBeam = 5*resol+2;
    [Xmap, Ymap] = meshgrid(-distBeam:1:distBeam, -distBeam:1:distBeam);
    Xmap = reshape(Xmap, 1, []);
    Ymap = reshape(Ymap, 1, []);

    % Initialise the state machine. 
    fsm = 'start';
    step = 'vision';%'navigation';
    interfsm = fsm;
    interstep = step;
    interval = 'nul';
    
    if strcmp(step, 'navigation') 
        maxSpeed = 30;
    else
        maxSpeed = 38;
    end
    
    
    % Make sure everything is settled before we start. 
    pause(2);
    
    %% Start simulation. 
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

        %% NAVIGATION
        if strcmp(step, 'navigation')            
            if strcmp(fsm, 'start')
                % Read data from the depth sensor, more often called the Hokuyo 
                % This function returns the set of points the Hokuyo saw in pts. contacts indicates, for each point, if it
                % corresponds to an obstacle (the ray the Hokuyo sent was interrupted by an obstacle, and was not allowed to
                % go to infinity without being stopped(stopped at a distance of 5m).
                trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
                worldHokuyo1 = homtrans(trf, [h.hokuyo1Pos(1); h.hokuyo1Pos(2); h.hokuyo1Pos(3)]);
                worldHokuyo2 = homtrans(trf, [h.hokuyo2Pos(1); h.hokuyo2Pos(2); h.hokuyo2Pos(3)]);
                % Use the sensor to detect the visible points, within the world frame. 
                [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf);
                
                % Parameters to update the map
                PositionUpdateMap = [youbotPos(1) youbotPos(2)]; 
                AngleUpdateMap = youbotEuler(3); 
                
                % Initialize map extremities.
                minMapNew = floor(resol*[min([worldHokuyo1(1), pts(1, :), worldHokuyo2(1)])...
                    min([worldHokuyo1(2), pts(2, :), worldHokuyo2(2)])]);
                maxMapNew = ceil(resol*[max([worldHokuyo1(1), pts(1, :), worldHokuyo2(1)])...
                    max([worldHokuyo1(2), pts(2, :), worldHokuyo2(2)])]);
                
                minMap = [round((maxMapNew(1) + minMapNew(1))/2) - 6*resol,...
                    round((maxMapNew(2) + minMapNew(2))/2) - 6*resol];
                maxMap = [round((maxMapNew(1) + minMapNew(1))/2) + 6*resol,...
                    round((maxMapNew(2) + minMapNew(2))/2) + 6*resol];
                 
                inMap = inpolygon(Xmap, Ymap,...
                    resol*[worldHokuyo1(1), pts(1, :), worldHokuyo2(1)]- ...
                    round(resol*youbotPos(1)),...
                    resol*[worldHokuyo1(2), pts(2, :), worldHokuyo2(2)]- ...
                    round(resol*youbotPos(2)));

                NewVisited = [transpose(Xmap(inMap) - minMap(1) + ...
                    round(resol*youbotPos(1)) + 1)...
                    transpose(Ymap(inMap) - minMap(2) + ...
                    round(resol*youbotPos(2)) + 1);...
                    (round(resol*worldHokuyo1(1))- minMap(1)+1)...
                    (round(resol*worldHokuyo1(2))- minMap(2)+1);...
                    (round(resol*youbotPos(1))- minMap(1)+1)...
                    (round(resol*youbotPos(2))- minMap(2)+1)];

                % Add visited area.
                for i = 1 : length(NewVisited)
                    totalMap(NewVisited(i,1), NewVisited(i,2)) = 0;
                end
                
                % Vector containing the position of the obstacles.
                NewObstacle = bsxfun(@minus, unique(round(resol*[transpose(pts(1, contacts)) ...
                    transpose(pts(2, contacts))]),'rows'), minMap-1); 
                
                % Add walls.
                for i = 1 : length(NewObstacle)
                    if totalMap(NewObstacle(i,1), NewObstacle(i,2)) ~= 2  
                        totalMap(NewObstacle(i,1), NewObstacle(i,2)) = 2;
                        if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) ~= 2
                            totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) = 3;
                        end
                        if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)) ~= 2
                            totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)) = 3;
                        end
                        if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)+1) ~= 2
                            totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)+1) = 3;
                        end
                        if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) ~= 2
                            totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) = 3;
                        end
                        if totalMap(NewObstacle(i,1), NewObstacle(i,2)-1) ~= 2
                            totalMap(NewObstacle(i,1), NewObstacle(i,2)-1) = 3;
                        end
                        if totalMap(NewObstacle(i,1), NewObstacle(i,2)+1) ~= 2
                            totalMap(NewObstacle(i,1), NewObstacle(i,2)+1) = 3;
                        end
                        if totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)-1) ~= 2
                            totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)-1) = 3;
                        end
                        if totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)) ~= 2
                            totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)) = 3;
                        end
                        if totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)+1) ~= 2
                            totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)+1) = 3;
                        end
                    end                   
                end
                
                % Plot of the total map.
                figure;
                imagesc(totalMap)
                axis equal
                    
                % begin the displacement by a rotation of180 degree
                trajectory = [(resol*youbotPos(1)-minMap(1)+1) (resol*youbotPos(2)-minMap(2)+1);...
                    (round(resol*(youbotPos(1)))-minMap(1)+1) (round(resol*(youbotPos(2)))-minMap(2)+1)];
                posTrajectory = 1;
                NewAngl = youbotEuler(3)+pi+0.05;
   
                fsm = 'rotate';
            
            % If initialisation is finished, update.
            else
               
                % Update afer a certain distance or a certain rotation
                if (sqrt((youbotPos(1)-PositionUpdateMap(1))^2 + (youbotPos(2)-PositionUpdateMap(2))^2)>= 5/(7*resol))...
                    ||(abs(angdiff(AngleUpdateMap - youbotEuler(3))) > (pi/5))

                    PositionUpdateMap = [youbotPos(1) youbotPos(2)];
                    AngleUpdateMap = youbotEuler(3);
                    
                    trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
                    worldHokuyo1 = homtrans(trf, [h.hokuyo1Pos(1); h.hokuyo1Pos(2); h.hokuyo1Pos(3)]);
                    worldHokuyo2 = homtrans(trf, [h.hokuyo2Pos(1); h.hokuyo2Pos(2); h.hokuyo2Pos(3)]);
                    % Use the sensor to detect the visible points, within the world frame. 
                    [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf);
                    %update map extremity
                    minMapNew = round(resol*[min([worldHokuyo1(1), pts(1, :), worldHokuyo2(1)])...
                        min([worldHokuyo1(2), pts(2, :), worldHokuyo2(2)])]) - 1;
                    maxMapNew = round(resol*[max([worldHokuyo1(1), pts(1, :), worldHokuyo2(1)])...
                        max([worldHokuyo1(2), pts(2, :), worldHokuyo1(2)])]) + 1;
                    % if map need to be modified
                    if (minMapNew(1) < minMap(1))||(minMapNew(2) < minMap(2))||...
                    (maxMapNew(1) > maxMap(1))||(maxMapNew(2) > maxMap(2))
                        % modify min x
                        if (minMapNew(1) < minMap(1))
                            minMapInter(1) = minMapNew(1) - 4*resol;
                        else
                            minMapInter(1) = minMap(1);
                        end
                        % modify min y
                        if (minMapNew(2) < minMap(2))
                            minMapInter(2) = minMapNew(2) - 4*resol;
                        else
                            minMapInter(2) = minMap(2);
                        end
                        % modify max x
                        if (maxMapNew(1) > maxMap(1))
                            maxMapInter(1) = maxMapNew(1) + 4*resol;
                        else
                            maxMapInter(1) = maxMap(1);
                        end
                        % modify max y
                        if (maxMapNew(2) > maxMap(2))
                            maxMapInter(2) = maxMapNew(2) + 4*resol;
                        else
                            maxMapInter(2) = maxMap(2);
                        end
                        %update totalMap
                        totalMap = horzcat(ones(size(totalMap,1), minMap(2) - minMapInter(2)),...
                            totalMap, ones(size(totalMap,1), maxMapInter(2) - maxMap(2)));
                        totalMap = vertcat(ones(minMap(1) - minMapInter(1), size(totalMap,2)),...
                            totalMap, ones(maxMapInter(1) - maxMap(1), size(totalMap,2)));
                        % update trajectory
                        trajectory = bsxfun(@plus,trajectory, minMap - minMapInter);
                        % update minMap and maxMap
                        minMap = minMapInter;
                        maxMap = maxMapInter;
                    end

                    % update walls
                    NewObstacle = bsxfun(@minus, unique(round(resol*[transpose(pts(1, contacts)) ...
                        transpose(pts(2, contacts))]),'rows'), minMap-1);

                    for i = 1 : length(NewObstacle)
                        if totalMap(NewObstacle(i,1), NewObstacle(i,2)) ~= 2  
                            totalMap(NewObstacle(i,1), NewObstacle(i,2)) = 2;
                            if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) ~= 2
                                totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) = 3;
                            end
                            if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)) ~= 2
                                totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)) = 3;
                            end
                            if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)+1) ~= 2
                                totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)+1) = 3;
                            end
                            if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) ~= 2
                                totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) = 3;
                            end
                            if totalMap(NewObstacle(i,1), NewObstacle(i,2)-1) ~= 2
                                totalMap(NewObstacle(i,1), NewObstacle(i,2)-1) = 3;
                            end
                            if totalMap(NewObstacle(i,1), NewObstacle(i,2)+1) ~= 2
                                totalMap(NewObstacle(i,1), NewObstacle(i,2)+1) = 3;
                            end
                            
                            if totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)-1) ~= 2
                                totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)-1) = 3;
                            end
                            if totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)) ~= 2
                                totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)) = 3;
                            end
                            if totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)+1) ~= 2
                                totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)+1) = 3;
                            end
                        end                   
                    end

                    %update visited area
                    inMap = inpolygon(Xmap, Ymap,...
                       resol*[worldHokuyo1(1), pts(1, :), worldHokuyo2(1)]- ...
                       round(resol*youbotPos(1)),...
                       resol*[worldHokuyo1(2), pts(2, :), worldHokuyo2(2)]- ...
                       round(resol*youbotPos(2)));

                    NewVisited = [transpose(Xmap(inMap) - minMap(1) + ...
                        round(resol*youbotPos(1)) + 1)...
                        transpose(Ymap(inMap) - minMap(2) + ...
                        round(resol*youbotPos(2)) + 1);...
                        (round(resol*worldHokuyo1(1))- minMap(1)+1)...
                        (round(resol*worldHokuyo1(2))- minMap(2)+1);...
                        (round(resol*youbotPos(1))- minMap(1)+1)...
                        (round(resol*youbotPos(2))- minMap(2)+1)];

                    %add visited area
                    for i = 1 : length(NewVisited)
                        if totalMap(NewVisited(i,1), NewVisited(i,2)) == 1
                            totalMap(NewVisited(i,1), NewVisited(i,2)) = 0;
                        end
                    end
                     
                elseif strcmp(fsm, 'rotate')   
                   
                    if(counter > 1)
                        if (posTrajectory == length(trajectory)+1)                            
                            NewAngl = atan(((resol*youbotPos(1)-minMap(1)+1)-point(1))/...
                                (point(2)-(resol*youbotPos(2)-minMap(2)+1)))+0.3;
                            if (point(2)-(resol*youbotPos(2)-minMap(2)+1)) > 0
                                NewAngl = NewAngl + pi;
                            end 
                        
                        rotateRightVel = angdiff(NewAngl, youbotEuler(3));
                        if (abs(angdiff(NewAngl, youbotEuler(3))) < .5 / 180 * pi) && ...
                            (abs(angdiff(prevOrientation, youbotEuler(3))) < .05 / 180 * pi)
                            rotateRightVel = 0;
                            fsm = 'newdestination';
                        end
                        end
                    end
                    
                    if posTrajectory ~= length(trajectory)+1
                    % The rotation velocity depends on the difference between the current angle and the target. 
                    rotateRightVel = 3*angdiff(NewAngl, youbotEuler(3));
                    
                    
                    % When the rotation is done (with a sufficiently high precision), move on to the next state. 
                    if (abs(angdiff(NewAngl, youbotEuler(3))) < .5 / 180 * pi) && ...
                            (abs(angdiff(prevOrientation, youbotEuler(3))) < .05 / 180 * pi)
                        rotateRightVel = 0;
                        fsm = 'drive';
                        
                        prevPosition = [youbotPos(1) youbotPos(2)];
                        interPos = [resol*youbotPos(1) resol*youbotPos(2)];
                        initialDist = sqrt((trajectory(posTrajectory,1) - (resol*youbotPos(1)-minMap(1)+1))^2 +...
                            (trajectory(posTrajectory,2) - (resol*youbotPos(2)-minMap(2)+1))^2);
                        
                        if counter == 1
                        if length(trajectory(:,1)) > 5                            
                            if NewGoal ~= [0 0] 
                                % Max des x
                                if NewGoal(2) == length(totalMap(:,1))
                                    % Max des y
                                    if NewGoal(1) == length(totalMap(1,:))
                                        if totalMap(NewGoal(2), NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)-1) ~= 1
                                            fsm = 'newdestination';                           
                                        end   
                                    else
                                        if totalMap(NewGoal(2), NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)-1) ~= 1
                                            fsm = 'newdestination';                           
                                        end   
                                    end                            
                                else
                                    % Max des y
                                    if NewGoal(1) == length(totalMap(1,:))
                                        if totalMap(NewGoal(2)+1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)+1, NewGoal(1)-1) ~= 1 &&...                                        
                                                totalMap(NewGoal(2), NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)-1) ~= 1
                                            fsm = 'newdestination';                           
                                        end   
                                    else 
                                        if totalMap(NewGoal(2)+1, NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2)+1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)+1, NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)-1) ~= 1
                                            fsm = 'newdestination';                           
                                        end                       
                                    end
                                end

                            end
                        elseif length(trajectory(:,1)) > 4 && posTrajectory >= 4  
                            if NewGoal ~= [0 0] 
                                % Max des x
                                if NewGoal(2) == length(totalMap(:,1))
                                    % Max des y
                                    if NewGoal(1) == length(totalMap(1,:))
                                        if totalMap(NewGoal(2), NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)-1) ~= 1
                                            fsm = 'newdestination';                           
                                        end   
                                    else
                                        if totalMap(NewGoal(2), NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)-1) ~= 1
                                            fsm = 'newdestination';                           
                                        end   
                                    end                            
                                else
                                    % Max des y
                                    if NewGoal(1) == length(totalMap(1,:))
                                        if totalMap(NewGoal(2)+1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)+1, NewGoal(1)-1) ~= 1 &&...                                        
                                                totalMap(NewGoal(2), NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)-1) ~= 1
                                            fsm = 'newdestination';                           
                                        end   
                                    else 
                                        if totalMap(NewGoal(2)+1, NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2)+1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)+1, NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)-1) ~= 1
                                            fsm = 'newdestination';                           
                                        end                       
                                    end
                                end
                            end 
                            
                        elseif length(trajectory(:,1)) > 3 && posTrajectory == 3  
                            if (NewGoal(1) ~= 0) && (NewGoal(2) ~= 0) 
                                % Max des x
                                if NewGoal(2) == length(totalMap(:,1))
                                    % Max des y
                                    if NewGoal(1) == length(totalMap(1,:))
                                        if totalMap(NewGoal(2), NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)-1) ~= 1
                                            fsm = 'newdestination';                           
                                        end   
                                    else
                                        if totalMap(NewGoal(2), NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)-1) ~= 1
                                            fsm = 'newdestination';                           
                                        end   
                                    end                            
                                else
                                    % Max des y
                                    if NewGoal(1) == length(totalMap(1,:))
                                        if totalMap(NewGoal(2)+1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)+1, NewGoal(1)-1) ~= 1 &&...                                        
                                                totalMap(NewGoal(2), NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)-1) ~= 1
                                            fsm = 'newdestination';                           
                                        end   
                                    else 
                                        if totalMap(NewGoal(2)+1, NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2)+1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)+1, NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2), NewGoal(1)-1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)+1) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)) ~= 1 &&...
                                                totalMap(NewGoal(2)-1, NewGoal(1)-1) ~= 1
                                            fsm = 'newdestination';                           
                                        end                       
                                    end
                                end
                            end                            
                        end
                        
                        end
                    end  
                    end
                    prevOrientation = youbotEuler(3);


                    
                    elseif strcmp(fsm, 'drive')                                                                                 
                    % DistPrev : distance between the position of the robot and the previous point.
                    % InitialDist : distance between the previous point and the next point.
                    DistPrev = sqrt((interPos(1) - resol*youbotPos(1))^2 +...
                                    (interPos(2) - resol*youbotPos(2))^2);
                   
                    % To satisfy conditions imposed by mechanics about the speed, we choose to accelerate 
                    % until a certain length called maxDist and after this length we keep a constant speed. 
                    % The decceleration use the same principle.                  
                    
                    if DistPrev <= initialDist   
                        if initialDist < 2*maxDist
                            middleDist = initialDist/2;
                            
                            % Acceleration.
                            forwBackVel = -maxSpeed/maxDist * DistPrev;

                            % Decceleration.
                            if DistPrev > middleDist
                                forwBackVel = -maxSpeed/maxDist * (maxDist-DistPrev);
                            end
                            
                        else
                            if DistPrev < maxDist
                                forwBackVel = -maxSpeed/maxDist * DistPrev;
                            elseif DistPrev < initialDist - maxDist
                                forwBackVel = -maxSpeed;
                            else
                                forwBackVel = -maxSpeed/maxDist * (maxDist-DistPrev);
                            end
                        end
                        if (DistPrev - initialDist) < 2*maxDist
                            middleDist = (DistPrev - initialDist)/2;
                            % Acceleration
                            forwBackVel = maxSpeed/maxDist * (maxDist - (DistPrev - initialDist));

                            % Decceleration
                            if (DistPrev - initialDist) < middleDist
                                forwBackVel = maxSpeed/maxDist * (DistPrev - initialDist);
                            end 
                        else                        
                            if (DistPrev - initialDist) < maxDist
                                forwBackVel = maxSpeed/maxDist * (DistPrev - initialDist);
                            elseif (DistPrev - initialDist) < (DistPrev - initialDist) - maxDist
                                forwBackVel = maxSpeed;
                            else
                                forwBackVel = maxSpeed/maxDist * (maxDist - (DistPrev - initialDist));
                            end
                        end
                    end
         
                    % If the goal is reached stop the robot. 8: length of
                    % the sensor
                    if (abs(initialDist - DistPrev) < 0.01) && (resol*sqrt((prevPosition(1)-youbotPos(1))^2 + ...
                            (prevPosition(2)-youbotPos(2))^2) < 0.005)
                        forwBackVel = 0;
                        
                        if posTrajectory == size(trajectory,1) && counter == 1
                            fsm ='newdestination'; 
                        elseif posTrajectory == size(trajectory,1) && counter > 1
                            fsm = 'rotate';
                            posTrajectory = posTrajectory +1;
                        else
                            posTrajectory = posTrajectory +1;
                            NewAngl = atan(((resol*youbotPos(1)-minMap(1)+1)-trajectory(posTrajectory,1))/...
                                (trajectory(posTrajectory,2)-(resol*youbotPos(2)-minMap(2)+1)));
                            if (trajectory(posTrajectory,2)-(resol*youbotPos(2)-minMap(2)+1)) > 0
                                NewAngl = NewAngl + pi;
                            end
                            prevOrientation = youbotEuler(3);
                            fsm = 'rotate';                       
                        end
                    end
                    prevPosition = [youbotPos(1) youbotPos(2)];
                        
                elseif strcmp(fsm, 'newdestination')
                    
                    figure;
                    imagesc(totalMap)
                    axis equal
                     
                    % Define new goal
                    totalBoundariesMap = sparse([],[],[],double(maxMap(1)-minMap(1)+1), double(maxMap(2)-minMap(2)+1),0);
                    for i = 2 : size(totalMap, 1)-1             
                        for j = 2 : size(totalMap, 2)-1               
                            % if visited point
                            if totalMap(i,j) == 0
                                %if in contact with not yet visited point
                                if((totalMap(i-1,j) == 1)||(totalMap(i+1,j) == 1)|| ...
                                        (totalMap(i,j-1) == 1)||(totalMap(i,j+1) == 1))
                                    totalBoundariesMap = totalBoundariesMap + ...
                                        sparse(double(i),double(j), 1, double(maxMap(1)-minMap(1)+1),...
                                        double(maxMap(2)-minMap(2)+1));
                                end
                            end
                        end
                    end
                    
                    % extract rows and colums from totalBoundariesMap
                    [remRow, remCol] = find(totalBoundariesMap == 1);
                    
                    % If the map is completely explored, navigation is finished.
                    if isempty(remRow)
                        fsm = 'finished';
                    else
                        NewStart = [resol*youbotPos(1)-minMap(1)+1, resol*youbotPos(2)-minMap(2)+1];
                        
                        % find destination with max distance for the first
                        % trajectory and min for the other
                        if initTraj
                            indexGoal = find(bsxfun(@hypot, remRow-NewStart(1), remCol-NewStart(2))==...
                                max(bsxfun(@hypot, remRow-NewStart(1), remCol-NewStart(2))),1,'first');
                            initTraj = false;
                        else
                            indexGoal = find(bsxfun(@hypot, remRow-NewStart(1), remCol-NewStart(2))==...
                                min(bsxfun(@hypot, remRow-NewStart(1), remCol-NewStart(2))),1,'first');
                        end
                     
                        NewGoal = [remCol(indexGoal) remRow(indexGoal)];
                        NewStart = [round(resol*(youbotPos(1)))-minMap(1)+1, round(resol*(youbotPos(2)))-minMap(2)+1];
                        % We chose Dstar because it is faster than the others.
                        %figure;
                        ds = Dstar(totalMap);    % create navigation object
                        ds.plan(NewGoal);       % create plan for specified goal
                        
                        trajectory = ds.path(flip(NewStart));     % animate path from this start location
                        %ds.path(flip(NewStart)) 

                        % to have x than y 
                        trajectory = flip(trajectory,2); 
                        if size(trajectory,1)>1
                            
                            trajectory2 = zeros(size(trajectory,1), 1);

                            if ((trajectory(1,1)- NewStart(1)) ~= (trajectory(2,1)-trajectory(1,1)))||...
                                    ((trajectory(1,2)- NewStart(2)) ~= (trajectory(2,2)-trajectory(1,2)))
                                    trajectory2(1) = 1;
                            end
                            for i = 2: length(trajectory)-1
                                if ((trajectory(i,1)-trajectory(i-1,1)) ~= (trajectory(i+1,1)-trajectory(i,1)))||...
                                    ((trajectory(i,2)-trajectory(i-1,2)) ~= (trajectory(i+1,2)-trajectory(i,2)))
                                    trajectory2(i) = 1;
                                end
                            end
                            trajectory2(end-1) = 1;
                            trajectory2(end) = 0;
                            trajectory = trajectory(trajectory2 == 1, :);
                        end
                        
                        posTrajectory = 1;
                        NewAngl = atan(((resol*youbotPos(1)-minMap(1)+1)-trajectory(1,1))/...
                            (trajectory(1,2)-(resol*youbotPos(2)-minMap(2)+1)));
                        if (trajectory(1,2)-(resol*youbotPos(2)-minMap(2)+1)) > 0
                            NewAngl = NewAngl + pi;
                        end  
                        fsm = 'rotate';
                    end        

                 % End of navigation.
                 elseif  strcmp(fsm, 'finished')  
                    pause(1);
                      
                    fsm = 'start';
                    if fullcode
                        step = 'vision';
                    else
                        step = 'finished';
                         % save the map obtain by the navigation
                        save('totalmap.mat','totalMap'); 
                        save('minmap.mat', 'minMap');
                        save('maxmap.mat', 'maxMap');
                    end
                    fprintf('Switching to step: %s\n', step);
                 else
                    fsm = 'finished';
                    error('Unknown state %s.', fsm);
                 end
            end
            
                
        elseif strcmp(step, 'travel') 
           
            if strcmp(fsm, 'rotate')
            % The rotation velocity depends on the difference between the current angle and the target. 
            rotateRightVel = angdiff(NewAngl, youbotEuler(3));
            % When the rotation is done (with a sufficiently high precision), move on to the next state. 
            if (abs(angdiff(NewAngl, youbotEuler(3))) < .5 / 180 * pi) && ...
                    (abs(angdiff(prevOrientation, youbotEuler(3))) < .05 / 180 * pi)
                rotateRightVel = 0;
                
                if strcmp(interval, 'only_rotate')
                    step = interstep;
                    fsm = interfsm;
                    interval = 'nul';
                else 
                fsm = 'drive';
                prevPosition = [youbotPos(1) youbotPos(2)];
                interPos = [resol*youbotPos(1) resol*youbotPos(2)];
                initialDist = sqrt((trajectory(posTrajectory,1) - (resol*youbotPos(1)-minMap(1)+1))^2 +...
                    (trajectory(posTrajectory,2) - (resol*youbotPos(2)-minMap(2)+1))^2);
                end
            end                 
            prevOrientation = youbotEuler(3);

            elseif strcmp(fsm, 'drive')
            % DistPrev : distance between the position of the robot and the previous point.
            % InitialDist : distance between the previous point and the next point.
            DistPrev = sqrt((interPos(1) - resol*youbotPos(1))^2 +...
                (interPos(2) - resol*youbotPos(2))^2);

            % To satisfy conditions imposed by mechanics about the speed, we choose to accelerate 
            % until a certain length called maxDist and after this length we keep a constant speed. 
            % The decceleration use the same principle.

            % If the distance is too short, we can't go to the maximum speed.
            if DistPrev <= initialDist   
                if initialDist < 2*maxDist
                    middleDist = initialDist/2;

                    % Acceleration.
                    forwBackVel = -maxSpeed/maxDist * DistPrev;

                    % Decceleration.
                    if DistPrev > middleDist
                        forwBackVel = -maxSpeed/maxDist * (maxDist-DistPrev);
                    end

                else
                    if DistPrev < maxDist
                        forwBackVel = -maxSpeed/maxDist * DistPrev;
                    elseif DistPrev < initialDist - maxDist
                        forwBackVel = -maxSpeed;
                    else
                        forwBackVel = -maxSpeed/maxDist * (maxDist-DistPrev);
                    end
                end
                if (DistPrev - initialDist) < 2*maxDist
                    middleDist = (DistPrev - initialDist)/2;
                    % Acceleration
                    forwBackVel = maxSpeed/maxDist * (maxDist - (DistPrev - initialDist));

                    % Decceleration
                    if (DistPrev - initialDist) < middleDist
                        forwBackVel = maxSpeed/maxDist * (DistPrev - initialDist);
                    end 
                else                        
                    if (DistPrev - initialDist) < maxDist
                        forwBackVel = maxSpeed/maxDist * (DistPrev - initialDist);
                    elseif (DistPrev - initialDist) < (DistPrev - initialDist) - maxDist
                        forwBackVel = maxSpeed;
                    else
                        forwBackVel = maxSpeed/maxDist * (maxDist - (DistPrev - initialDist));
                    end
                end
            end

            % If the goal is reached stop the robot.
            if (abs(initialDist - DistPrev) < 0.01) && (resol*sqrt((prevPosition(1)-youbotPos(1))^2 + ...
                    (prevPosition(2)-youbotPos(2))^2) < 0.005)
                forwBackVel = 0;
                % end of the trajectory
                if (posTrajectory == size(trajectory,1))||(strcmp(interval, 'only_drive'))
                    step = interstep;
                    fsm = interfsm;
                else
                    posTrajectory = posTrajectory +1;
                    NewAngl = atan(((resol*youbotPos(1)-minMap(1)+1)-trajectory(posTrajectory,1))/...
                        (trajectory(posTrajectory,2)-(resol*youbotPos(2)-minMap(2)+1)));
                    if (trajectory(posTrajectory,2)-(resol*youbotPos(2)-minMap(2)+1)) > 0
                        NewAngl = NewAngl + pi;
                    end
                    prevOrientation = youbotEuler(3);
                    fsm = 'rotate';                       
                end
            end
            prevPosition = [youbotPos(1) youbotPos(2)];

            elseif strcmp(fsm, 'newdestination')
                
                % destination need to be define previously
                NewStart = [round(resol*(youbotPos(1)))-minMap(1)+1, round(resol*(youbotPos(2)))-minMap(2)+1];

                % We chose Dstar because it is faster than the others.
                %figure;
                ds = Dstar(totalMap);    % create navigation object
                ds.plan(flip(NewGoal));       % create plan for specified goal
                trajectory = ds.path(flip(NewStart));     % animate path from this start location
                %ds.path(flip(NewStart)) 

                % to have x than y 
                trajectory = flip(trajectory,2);
                    
                if size(trajectory,1) > 1
                    % create the trajectory to the destination
                    trajectory2 = zeros(size(trajectory,1), 1);

                    if ((trajectory(1,1)- NewStart(1)) ~= (trajectory(2,1)-trajectory(1,1)))||...
                            ((trajectory(1,2)- NewStart(2)) ~= (trajectory(2,2)-trajectory(1,2)))
                            trajectory2(1) = 1;
                    end
                    trajectorycounter  = 0;
                    for i = 2: length(trajectory)-1
                        trajectorycounter  = trajectorycounter +1;
                        if ((trajectory(i,1)-trajectory(i-1,1)) ~= (trajectory(i+1,1)-trajectory(i,1)))||...
                            ((trajectory(i,2)-trajectory(i-1,2)) ~= (trajectory(i+1,2)-trajectory(i,2)))
                            trajectory2(i) = 1;
                            trajectorycounter  = 0;
                        end
                        if trajectorycounter == 10
                            trajectory2(i) = 1;
                            trajectorycounter  = 0;
                        end      
                    end
                    %trajectory2(end-1) = 1;
                    trajectory2(end) = 1;
                    trajectory = trajectory(trajectory2 == 1, :);
                end
                
                %already at the destination
                 if size(trajectory,1) == 0
                    step = interstep;
                    fsm = interfsm;
                 else
                    posTrajectory = 1;
                    NewAngl = atan(((resol*youbotPos(1)-minMap(1)+1)-trajectory(1,1))/...
                        (trajectory(1,2)-(resol*youbotPos(2)-minMap(2)+1)));
                    if (trajectory(1,2)-(resol*youbotPos(2)-minMap(2)+1)) > 0
                        NewAngl = NewAngl + pi;
                    end  
                    fsm = 'rotate';
                 end
            end  
            
        elseif strcmp(step, 'vision') 
            
            if strcmp(fsm, 'start')
                
                interstep = 'vision';
                fprintf('Switching to step: %s\n', step);
               
                if ~fullcode  
                    % save map and boundary value to avoid to run navigation that
                    % is very long (~8min)
                    Map = load('totalmap.mat');
                    totalMap = cell2mat(struct2cell(Map));
                    Map = load('minmap.mat');
                    minMap = cell2mat(struct2cell(Map));
                    Map = load('maxmap.mat');
                    maxMap = cell2mat(struct2cell(Map));

                    figure;
                    imagesc(totalMap)
                    axis equal
                end
                %turn off the hokuyo captor
                res = vrep.simxSetIntegerSignal(h.id, 'handle_xy_sensor', 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res);   
                

            %% Vision Part
                totalMap2 = totalMap;
                k = 1;
                counter = 1;

                %-------------------------Take the limit of the map------------------------
                Uplim = inf;
                Leftlim = inf;
                Downlim = -inf;
                Rightlim = -inf;

                for i=1:length(totalMap(:,1))
                    for j=1:length(totalMap(1,:))
                        if(totalMap(i,j) ~= 1)
                            if Uplim > i
                                Uplim = i;
                            end
                            if Downlim < i
                                Downlim = i;
                            end
                            if Leftlim > j
                                Leftlim = j;
                            end
                            if Rightlim < j
                                Rightlim = j;
                            end
                        end
                    end
                end

                % -----------------------Take some potential target-----------------------
                % We have to add or soustract 3 on the limit because it's the width of the
                % walls. Firstly we make a vertical scanning.

                for i=Uplim+3:Downlim-3
                    j = Leftlim+3;
                    while (j <= Rightlim-3)
                        if totalMap(i,j) ~= 0
                            % Vertical wall.
                            if (totalMap(i,j) == 3 && totalMap(i,j+1) == 2 && totalMap(i,j+2) == 3 && totalMap(i,j+3) == 0)
                                j = j+3 ;  
                            % Side of horizontal wall.
                            elseif totalMap(i,j) == 3 && totalMap(i,j+1) == 3 && totalMap(i,j+2) == 3 && ...
                                    totalMap(i,j+3) == 3 && totalMap(i,j+4) == 3 && totalMap(i,j+5) == 3
                                j = j+5;
                            % Inside of horizontal wall.
                            elseif totalMap(i,j) == 2 && totalMap(i,j+1) == 2 && totalMap(i,j+2) == 2 && ...
                                    totalMap(i,j+3) == 2 && totalMap(i,j+4) == 2 && totalMap(i,j+5) == 2
                                j = j+5;
                            % Target.
                            else
                                tab(k,1) = i;
                                tab(k,2) = j;
                                k = k+1;
                            end   
                        end
                        j = j+1;
                    end 
                end

                % ---------------------------Erase false target---------------------------
                % We look if the target are a part of the wall or not. If it is the case,
                % we erase it from the all target.

                k = 1;
                for i=1:length(tab)
                    % Inside of a wall.
                    if totalMap(tab(i,1),tab(i,2)) == 2
                        if totalMap(tab(i,1),tab(i,2)-1) == 3 && totalMap(tab(i,1),tab(i,2)+1) == 3
                            if totalMap(tab(i,1)-1,tab(i,2)-1) == 3 && totalMap(tab(i,1)-1,tab(i,2)+1) == 3 ...
                                    && totalMap(tab(i,1)-1,tab(i,2)) == 2 && totalMap(tab(i,1)-2,tab(i,2)-1) == 3 ...
                                    && totalMap(tab(i,1)-2,tab(i,2)+1) == 3 && totalMap(tab(i,1)-2,tab(i,2)) == 2 ...
                                    && totalMap(tab(i,1)-3,tab(i,2)-1) == 3 && totalMap(tab(i,1)-3,tab(i,2)+1) == 3 ...
                                    && totalMap(tab(i,1)-3,tab(i,2)) == 2 && totalMap(tab(i,1)-4,tab(i,2)) == 2               
                                for l=0:4
                                    tab2(k,1) = tab(i,1)-l;
                                    tab2(k,2) = tab(i,2)-1;
                                    k = k +1;
                                    tab2(k,1) = tab(i,1)-l;
                                    tab2(k,2) = tab(i,2)+1;
                                    k = k +1;
                                    tab2(k,1) = tab(i,1)-l;
                                    tab2(k,2) = tab(i,2);
                                    k = k +1;                    
                                end
                            elseif totalMap(tab(i,1)+1,tab(i,2)-1) == 3 && totalMap(tab(i,1)+1,tab(i,2)+1) == 3 ...
                                    && totalMap(tab(i,1)+1,tab(i,2)) == 2 && totalMap(tab(i,1)+2,tab(i,2)-1) == 3 ...
                                    && totalMap(tab(i,1)+2,tab(i,2)+1) == 3 && totalMap(tab(i,1)+2,tab(i,2)) == 2 ...
                                    && totalMap(tab(i,1)+3,tab(i,2)-1) == 3 && totalMap(tab(i,1)+3,tab(i,2)+1) == 3 ...
                                    && totalMap(tab(i,1)+3,tab(i,2)) == 2 && totalMap(tab(i,1)+4,tab(i,2)) == 2
                                for l=0:4
                                    tab2(k,1) = tab(i,1)+l;
                                    tab2(k,2) = tab(i,2)-1;
                                    k = k +1;
                                    tab2(k,1) = tab(i,1)+l;
                                    tab2(k,2) = tab(i,2)+1;
                                    k = k +1;
                                    tab2(k,1) = tab(i,1)+l;
                                    tab2(k,2) = tab(i,2);
                                    k = k +1;                    
                                end
                            end            
                        end   
                    % Side of the wall.
                    elseif totalMap(tab(i,1),tab(i,2)) == 3
                        if totalMap(tab(i,1),tab(i,2)-1) == 3 && totalMap(tab(i,1),tab(i,2)+1) == 3
                            if totalMap(tab(i,1)-1,tab(i,2)-1) == 3 && totalMap(tab(i,1)-1,tab(i,2)+1) == 3 ...
                                    && totalMap(tab(i,1)-1,tab(i,2)) == 2 && totalMap(tab(i,1)-2,tab(i,2)-1) == 3 ...
                                    && totalMap(tab(i,1)-2,tab(i,2)+1) == 3 && totalMap(tab(i,1)-2,tab(i,2)) == 2 ...
                                    && totalMap(tab(i,1)-3,tab(i,2)-1) == 3 && totalMap(tab(i,1)-3,tab(i,2)+1) == 3 ...
                                    && totalMap(tab(i,1)-3,tab(i,2)) == 2 && totalMap(tab(i,1)-4,tab(i,2)) == 2
                                for l=0:4
                                    tab2(k,1) = tab(i,1)-l;
                                    tab2(k,2) = tab(i,2)-1;
                                    k = k +1;
                                    tab2(k,1) = tab(i,1)-l;
                                    tab2(k,2) = tab(i,2)+1;
                                    k = k +1;
                                    tab2(k,1) = tab(i,1)-l;
                                    tab2(k,2) = tab(i,2);
                                    k = k +1;                    
                                end
                            elseif totalMap(tab(i,1)+1,tab(i,2)-1) == 3 && totalMap(tab(i,1)+1,tab(i,2)+1) == 3 && ...
                                    totalMap(tab(i,1)+1,tab(i,2)) == 2 && totalMap(tab(i,1)+2,tab(i,2)-1) == 3 && ...
                                    totalMap(tab(i,1)+2,tab(i,2)+1) == 3 && totalMap(tab(i,1)+2,tab(i,2)) == 2 ...
                                    && totalMap(tab(i,1)+3,tab(i,2)-1) == 3 && totalMap(tab(i,1)+3,tab(i,2)+1) == 3 ...
                                    && totalMap(tab(i,1)+3,tab(i,2)) == 2 && totalMap(tab(i,1)+4,tab(i,2)) == 2                
                                for l=0:4
                                    tab2(k,1) = tab(i,1)+l;
                                    tab2(k,2) = tab(i,2)-1;
                                    k = k +1;
                                    tab2(k,1) = tab(i,1)+l;
                                    tab2(k,2) = tab(i,2)+1;
                                    k = k +1;
                                    tab2(k,1) = tab(i,1)+l;
                                    tab2(k,2) = tab(i,2);
                                    k = k +1;                    
                                end
                            end            
                        end
                    end    
                end

                % We erase the false target.
                for i=1:length(tab2)
                    FalseTarg = find(tab(:,1) == tab2(i,1) & tab(:,2) == tab2(i,2));
                    tab(FalseTarg,:) = [];
                    clear('tab()');
                end

                % -----------------------Take some potential target-----------------------
                % We have to add or soustract 3 on the limit because it's the width of the
                % walls. Now we make an horizontal scanning.

                k = 1;
                j = Leftlim+3;
                while j <= Rightlim-3
                    i = Uplim+3;
                    while (i <= Downlim-3)
                        if totalMap(i,j) ~= 0
                            % Vertical wall.
                            if (totalMap(i,j) == 3 && totalMap(i+1,j) == 2 && totalMap(i+2,j) == 3 && totalMap(i+3,j) == 0)
                                i = i+3 ;  
                            % Side of horizontal wall.
                            elseif totalMap(i,j) == 3 && totalMap(i+1,j) == 3 && totalMap(i+2,j) == 3 && ...
                                    totalMap(i+3,j) == 3 && totalMap(i+4,j) == 3 && totalMap(i+5,j) == 3
                                i = i+5;
                            % Inside of horizontal wall.
                            elseif totalMap(i,j) == 2 && totalMap(i+1,j) == 2 && totalMap(i+2,j) == 2 && ...
                                    totalMap(i+3,j) == 2 && totalMap(i+4,j) == 2 && totalMap(i+5,j) == 2
                                i = i+5;
                            % Target.
                            else
                                tab3(k,1) = i;
                                tab3(k,2) = j;
                                k = k+1;
                            end   
                        end
                        i = i+1;
                    end 
                    j = j+1;
                end

                % ---------------------------Erase false target---------------------------
                % We look if the target are a part of the wall or not. If it is the case,
                % we erase it from the all target.

                k = 1;
                for i=1:length(tab3)
                    if totalMap(tab3(i,1),tab3(i,2)) == 2
                        if totalMap(tab3(i,1)-1,tab3(i,2)) == 3 && totalMap(tab3(i,1)+1,tab3(i,2)) == 3
                            if totalMap(tab3(i,1)-1,tab3(i,2)-1) == 3 && totalMap(tab3(i,1)+1,tab3(i,2)-1) == 3 ...
                                    && totalMap(tab3(i,1),tab3(i,2)-1) == 2 && totalMap(tab3(i,1)-1,tab3(i,2)-2) == 3 ...
                                    && totalMap(tab3(i,1)+1,tab3(i,2)-2) == 3 && totalMap(tab3(i,1),tab3(i,2)-2) == 2 ...
                                    && totalMap(tab3(i,1)-1,tab3(i,2)-3) == 3 && totalMap(tab3(i,1)+1,tab3(i,2)-3) == 3 ...
                                    && totalMap(tab3(i,1),tab3(i,2)-3) == 2 && totalMap(tab3(i,1),tab3(i,2)-4) == 2
                                for l=0:4
                                    tab4(k,1) = tab3(i,1)-1;
                                    tab4(k,2) = tab3(i,2)-l;
                                    k = k +1;
                                    tab4(k,1) = tab3(i,1)+1;
                                    tab4(k,2) = tab3(i,2)-l;
                                    k = k +1;
                                    tab4(k,1) = tab3(i,1);
                                    tab4(k,2) = tab3(i,2)-l;
                                    k = k +1;                    
                                end
                            elseif totalMap(tab3(i,1)-1,tab3(i,2)+1) == 3 && totalMap(tab3(i,1)+1,tab3(i,2)+1) == 3 ...
                                    && totalMap(tab3(i,1),tab3(i,2)+1) == 2 && totalMap(tab3(i,1)-1,tab3(i,2)+2) == 3 ...
                                    && totalMap(tab3(i,1)+1,tab3(i,2)+2) == 3 && totalMap(tab3(i,1),tab3(i,2)+2) == 2 ...
                                    && totalMap(tab3(i,1)-1,tab3(i,2)+3) == 3 && totalMap(tab3(i,1)+1,tab3(i,2)+2) == 3 ...
                                    && totalMap(tab3(i,1),tab3(i,2)+3) == 2 && totalMap(tab3(i,1),tab3(i,2)+4) == 2
                                for l=0:4
                                    tab4(k,1) = tab3(i,1)-1;
                                    tab4(k,2) = tab3(i,2)+l;
                                    k = k +1;
                                    tab4(k,1) = tab3(i,1)+1;
                                    tab4(k,2) = tab3(i,2)+l;
                                    k = k +1;
                                    tab4(k,1) = tab3(i,1);
                                    tab4(k,2) = tab3(i,2)+l;
                                    k = k +1;                    
                                end
                            end            
                        end   
                    elseif totalMap(tab3(i,1),tab3(i,2)) == 3
                        if totalMap(tab3(i,1)-1,tab3(i,2)) == 3 && totalMap(tab3(i,1)+1,tab3(i,2)) == 3
                            if totalMap(tab3(i,1)-1,tab3(i,2)-1) == 3 && totalMap(tab3(i,1)+1,tab3(i,2)-1) == 3 ...
                                    && totalMap(tab3(i,1),tab3(i,2)-1) == 2 && totalMap(tab3(i,1)-1,tab3(i,2)-2) == 3 ...
                                    && totalMap(tab3(i,1)+1,tab3(i,2)-2) == 3 && totalMap(tab3(i,1),tab3(i,2)-2) == 2 ...
                                    && totalMap(tab3(i,1)-1,tab3(i,2)-3) == 3 && totalMap(tab3(i,1)+1,tab3(i,2)-3) == 3 ...
                                    && totalMap(tab3(i,1),tab3(i,2)-3) == 2 && totalMap(tab3(i,1),tab3(i,2)-4) == 2
                                for l=0:4
                                    tab4(k,1) = tab3(i,1)-1;
                                    tab4(k,2) = tab3(i,2)-l;
                                    k = k +1;
                                    tab4(k,1) = tab3(i,1)+1;
                                    tab4(k,2) = tab3(i,2)-l;
                                    k = k +1;
                                    tab4(k,1) = tab3(i,1);
                                    tab4(k,2) = tab3(i,2)-l;
                                    k = k +1;                    
                                end
                            elseif totalMap(tab3(i,1)-1,tab3(i,2)+1) == 3 && totalMap(tab3(i,1)+1,tab3(i,2)+1) == 3 ...
                                    && totalMap(tab3(i,1),tab3(i,2)+1) == 2 && totalMap(tab3(i,1)-1,tab3(i,2)+2) == 3 ...
                                    && totalMap(tab3(i,1)+1,tab3(i,2)+2) == 3 && totalMap(tab3(i,1),tab3(i,2)+2) == 2 ...
                                    && totalMap(tab3(i,1)-1,tab3(i,2)+3) == 3 && totalMap(tab3(i,1)+1,tab3(i,2)+3) == 3 ...
                                    && totalMap(tab3(i,1),tab3(i,2)+3) == 2 && totalMap(tab3(i,1),tab3(i,2)+4) == 2
                                for l=0:4
                                    tab4(k,1) = tab3(i,1)-1;
                                    tab4(k,2) = tab3(i,2)+l;
                                    k = k +1;
                                    tab4(k,1) = tab3(i,1)+1;
                                    tab4(k,2) = tab3(i,2)+l;
                                    k = k +1;
                                    tab4(k,1) = tab3(i,1);
                                    tab4(k,2) = tab3(i,2)+l;
                                    k = k +1;                    
                                end 
                            end
                        end
                    end    
                end

                % We erase the false target.
                for i=1:length(tab4)
                    FalseTarg2 = find(tab3(:,1) == tab4(i,1) & tab3(:,2) == tab4(i,2));
                    tab3(FalseTarg2,:) = [];
                    clear('tab3()');
                end

                % ----------------------Erase the last false target-----------------------
                Target = intersect(tab,tab3,'rows');

                for i=1:length(Target)
                    totalMap(Target(i,1),Target(i,2)) = 4;   
                end

                for i=Uplim+3:Downlim-3
                    for j=Leftlim+3:Rightlim-3
                        if totalMap(i,j) == 4
                            if (totalMap(i+1,j) ~= 4 && totalMap(i-1,j) ~=4) || (totalMap(i,j+1) ~= 4 && totalMap(i,j-1) ~=4)
                                FalseTarg3 = find(Target(:,1) == i & Target(:,2) == j);
                                Target(FalseTarg3,:) = [];
                                clear('Target()');
                            end
                        end       
                    end 
                end

                % Update.
                totalMap = totalMap2;
                for i=1:length(Target)
                    totalMap(Target(i,1),Target(i,2)) = 4;   
                end

                i = 1;
                while i <= length(Target)
                    count = 0;
                    if totalMap(Target(i,1)+1, Target(i,2)) == 4
                        if totalMap(Target(i,1)+2, Target(i,2)) == 2
                            if totalMap(Target(i,1)-1, Target(i,2)) == 2 || totalMap(Target(i,1)-1, Target(i,2)) == 3
                                Target(i,:) = [];
                                clear('Target()');
                                count = count +1;                 
                            end        
                        elseif totalMap(Target(i,1)+2, Target(i,2)) == 3
                            if totalMap(Target(i,1)-1, Target(i,2)) == 2 || totalMap(Target(i,1)-1, Target(i,2)) == 3
                                Target(i,:) = [];
                                clear('Target()');
                                count = count +1;                 
                            end
                        end
                    end

                    if totalMap(Target(i,1), Target(i,2)+1) == 4
                        if totalMap(Target(i,1), Target(i,2)+2) == 2
                            if totalMap(Target(i,1), Target(i,2)-1) == 2 || totalMap(Target(i,1), Target(i,2)-1) == 3
                                Target(i,:) = [];
                                clear('Target()');
                                count = count +1;                 
                            end        
                        elseif totalMap(Target(i,1), Target(i,2)+2) == 3
                            if totalMap(Target(i,1), Target(i,2)-1) == 2 || totalMap(Target(i,1), Target(i,2)-1) == 3
                                Target(i,:) = [];
                                clear('Target()');
                                count = count +1;                 
                            end
                        end
                    end

                    if totalMap(Target(i,1)-1, Target(i,2)) == 4
                        if totalMap(Target(i,1)-2, Target(i,2)) == 2
                            if totalMap(Target(i,1)+1, Target(i,2)) == 2 || totalMap(Target(i,1)+1, Target(i,2)) == 3
                                Target(i,:) = [];
                                clear('Target()');
                                count = count +1;                 
                            end        
                        elseif totalMap(Target(i,1)-2, Target(i,2)) == 3
                            if totalMap(Target(i,1)+1, Target(i,2)) == 2 || totalMap(Target(i,1)+1, Target(i,2)) == 3
                                Target(i,:) = [];
                                clear('Target()');
                                count = count +1;                 
                            end
                        end
                    end

                    if totalMap(Target(i,1), Target(i,2)-1) == 4
                        if totalMap(Target(i,1), Target(i,2)-2) == 2
                            if totalMap(Target(i,1), Target(i,2)+1) == 2 || totalMap(Target(i,1), Target(i,2)+1) == 3
                                Target(i,:) = [];
                                clear('Target()');
                                count = count +1;                 
                            end        
                        elseif totalMap(Target(i,1), Target(i,2)-2) == 3
                            if totalMap(Target(i,1), Target(i,2)+1) == 2 || totalMap(Target(i,1), Target(i,2)+1) == 3
                                Target(i,:) = [];
                                clear('Target()');
                                count = count +1;                 
                            end
                        end
                    end
                    i = i+1-count;
                end

                % Update.
                totalMap = totalMap2;
                for i=1:length(Target)
                    totalMap(Target(i,1),Target(i,2)) = 4;   
                end

                % ----------------------------Find the baskets----------------------------
                % For each baskets, we check if there is a target at a distance less than
                % 1.

                % Basket 1.
                Panier1(1,1) = Target(1,1);
                Panier1(1,2) = Target(1,2);
                Target(1,:) = [];
                clear('Target()');
                k = 2;
                count = 1;
                z = 1;
                while count==1 || z <= length(Panier1)
                    count = 0;
                    for i=1:length(Target)
                        if sqrt((Target(i,1)-Panier1(z,1))^2 + (Target(i,2)-Panier1(z,2))^2) <= sqrt(2)
                            count = 1;
                            Panier1(k,1) = Target(i,1);
                            Panier1(k,2) = Target(i,2);
                            k = k+1;
                        end
                    end
                    for i=1:length(Panier1)
                        FalseTarg4 = find(Target(:,1) == Panier1(i,1) & Target(:,2) == Panier1(i,2));
                        Target(FalseTarg4,:) = [];
                        clear('Target()');   
                    end
                    z = z+1;
                end

                for i=1:length(Panier1)
                    totalMap(Panier1(i,1), Panier1(i,2)) = 5;    
                end

                % Basket 2.
                Panier2(1,1) = Target(1,1);
                Panier2(1,2) = Target(1,2);
                Target(1,:) = [];
                clear('Target()');
                k = 2;
                count = 1;
                z = 1;
                while count==1 || z <= length(Panier2)
                    count = 0;
                    for i=1:length(Target)
                        if sqrt((Target(i,1)-Panier2(z,1))^2 + (Target(i,2)-Panier2(z,2))^2) <= sqrt(2)
                            count = 1;
                            Panier2(k,1) = Target(i,1);
                            Panier2(k,2) = Target(i,2);
                            k = k+1;
                        end
                    end
                    for i=1:length(Panier2)
                        FalseTarg4 = find(Target(:,1) == Panier2(i,1) & Target(:,2) == Panier2(i,2));
                        Target(FalseTarg4,:) = [];
                        clear('Target()');   
                    end
                    z = z+1;
                end

                for i =1:length(Panier2)
                    totalMap(Panier2(i,1), Panier2(i,2)) = 6;    
                end

                % Basket 3.
                Panier3(1,1) = Target(1,1);
                Panier3(1,2) = Target(1,2);
                Target(1,:) = [];
                clear('Target()');
                k = 2;
                count = 1;
                z = 1;
                while count==1 || z <= length(Panier3)
                    count = 0;
                    for i=1:length(Target)
                        if sqrt((Target(i,1)-Panier3(z,1))^2 + (Target(i,2)-Panier3(z,2))^2) <= sqrt(2)
                            count = 1;
                            Panier3(k,1) = Target(i,1);
                            Panier3(k,2) = Target(i,2);
                            k = k+1;
                        end
                    end
                    for i=1:length(Panier3)
                        FalseTarg4 = find(Target(:,1) == Panier3(i,1) & Target(:,2) == Panier3(i,2));
                        Target(FalseTarg4,:) = [];
                        clear('Target()');   
                    end
                    z = z+1;
                end

                for i =1:length(Panier3)
                    totalMap(Panier3(i,1), Panier3(i,2)) = 7;    
                end

                % Basket 4.
                Panier4(1,1) = Target(1,1);
                Panier4(1,2) = Target(1,2);
                Target(1,:) = [];
                clear('Target()');
                k = 2;
                count = 1;
                z = 1;
                while count==1 || z <= length(Panier4)
                    count = 0;
                    for i=1:length(Target)
                        if sqrt((Target(i,1)-Panier4(z,1))^2 + (Target(i,2)-Panier4(z,2))^2) <= sqrt(2)
                            count = 1;
                            Panier4(k,1) = Target(i,1);
                            Panier4(k,2) = Target(i,2);
                            k = k+1;
                        end
                    end
                    for i=1:length(Panier4)
                        FalseTarg4 = find(Target(:,1) == Panier4(i,1) & Target(:,2) == Panier4(i,2));
                        Target(FalseTarg4,:) = [];
                        clear('Target()');   
                    end
                    z = z+1;
                end

                for i =1:length(Panier4)
                    totalMap(Panier4(i,1), Panier4(i,2)) = 8;    
                end

                % Basket 5.
                Panier5(1,1) = Target(1,1);
                Panier5(1,2) = Target(1,2);
                Target(1,:) = [];
                clear('Target()');
                k = 2;
                count = 1;
                z = 1;
                while count==1 || z <= length(Panier5)
                    count = 0;
                    for i=1:length(Target)
                        if sqrt((Target(i,1)-Panier5(z,1))^2 + (Target(i,2)-Panier5(z,2))^2) <= sqrt(2)
                            count = 1;
                            Panier5(k,1) = Target(i,1);
                            Panier5(k,2) = Target(i,2);
                            k = k+1;
                        end
                    end
                    for i=1:length(Panier5)
                        FalseTarg4 = find(Target(:,1) == Panier5(i,1) & Target(:,2) == Panier5(i,2));
                        Target(FalseTarg4,:) = [];
                        clear('Target()');   
                    end
                    z = z+1;
                end

                for i =1:length(Panier5)
                    totalMap(Panier5(i,1), Panier5(i,2)) = 9;    
                end

                % Basket 6.
                Panier6(1,1) = Target(1,1);
                Panier6(1,2) = Target(1,2);
                Target(1,:) = [];
                clear('Target()');
                k = 2;
                count = 1;
                z = 1;
                while count==1 || z <= length(Panier6)
                    count = 0;
                    for i=1:length(Target)
                        if sqrt((Target(i,1)-Panier6(z,1))^2 + (Target(i,2)-Panier6(z,2))^2) <= sqrt(2)
                            count = 1;
                            Panier6(k,1) = Target(i,1);
                            Panier6(k,2) = Target(i,2);
                            k = k+1;
                        end
                    end
                    for i=1:length(Panier6)
                        FalseTarg4 = find(Target(:,1) == Panier6(i,1) & Target(:,2) == Panier6(i,2));
                        Target(FalseTarg4,:) = [];
                        clear('Target()');   
                    end
                    z = z+1;
                end

                for i =1:length(Panier6)
                    totalMap(Panier6(i,1), Panier6(i,2)) = 10;    
                end

                % Basket 7.
                Panier7 = Target;
                for i =1:length(Panier7)
                    totalMap(Panier7(i,1), Panier7(i,2)) = 4;    
                end

                % ----------------------------Find the tables----------------------------
                % The tables are the bigger entities.

                % First table.
                w = length(Panier1);
                if w > length(Panier2)
                    if w > length(Panier3)
                        if w > length(Panier4)
                            if w > length(Panier5)
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier1;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier7)];%BrowniesA
                                    end
                                end                
                            else
                                w = length(Panier5);
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier5;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier1) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end  
                            end            
                        else
                            w = length(Panier4);
                            if w > length(Panier5)
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier4;
                                        T = [length(Panier2) length(Panier3) length(Panier1)...
                                            length(Panier5) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end                
                            else
                                w = length(Panier5);
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier5;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier1) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end  
                            end             
                        end        
                    else
                        w = length(Panier3);
                        if w > length(Panier4)
                            if w > length(Panier5)
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier3;
                                        T = [length(Panier2) length(Panier1) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end                
                            else
                                w = length(Panier5);
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier5;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier1) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end  
                            end            
                        else
                            w = length(Panier4);
                            if w > length(Panier5)
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier4;
                                        T = [length(Panier2) length(Panier3) length(Panier1)...
                                            length(Panier5) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end                
                            else
                                w = length(Panier5);
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier5;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier1) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end  
                            end 
                        end
                    end   
                else
                    w = length(Panier2);
                    if w > length(Panier3)
                        if w > length(Panier4)
                            if w > length(Panier5)
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier2;
                                        T = [length(Panier1) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end                
                            else
                                w = length(Panier5);
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier5;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier1) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end  
                            end            
                        else
                            w = length(Panier4);
                            if w > length(Panier5)
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier4;
                                        T = [length(Panier2) length(Panier3) length(Panier1)...
                                            length(Panier5) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end                
                            else
                                w = length(Panier5);
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier5;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier1) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end  
                            end             
                        end        
                    else
                        w = length(Panier3);
                        if w > length(Panier4)
                            if w > length(Panier5)
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier3;
                                        T = [length(Panier2) length(Panier1) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end                
                            else
                                w = length(Panier5);
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier5;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier1) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end  
                            end            
                        else
                            w = length(Panier4);
                            if w > length(Panier5)
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier4;
                                        T = [length(Panier2) length(Panier3) length(Panier1)...
                                            length(Panier5) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end                
                            else
                                w = length(Panier5);
                                if w > length(Panier6)
                                    if w > length(Panier7)
                                        Table1 = Panier5;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier1) length(Panier6) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                else
                                    w = length(Panier6);
                                    if w > length(Panier7)
                                        Table1 = Panier6;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier1) length(Panier7)];
                                    else
                                        Table1 = Panier7;
                                        T = [length(Panier2) length(Panier3) length(Panier4)...
                                            length(Panier5) length(Panier6) length(Panier1)];
                                    end
                                end  
                            end 
                        end
                    end    
                end

                % Second table.
                % We look for the biggest size.
                w = T(1);

                if w > T(2)
                    if w >T(3)
                        if w > T(4)
                            if w > T(5)
                                if w > T(6)
                                    Table2 = T(1);
                                else
                                    Table2 = T(6);
                                end                
                            else
                                w = T(5);
                                if w > T(6)
                                    Table2 = T(5);
                                else
                                    Table2 = T(6);
                                end  
                            end            
                        else
                            w = T(4);
                            if w > T(5)
                                if w > T(6)
                                    Table2 = T(4);
                                else
                                    Table2 = T(6);
                                end                
                            else
                                w = T(5);
                                if w > T(6)
                                    Table2 = T(5);
                                else
                                    Table2 = T(6);
                                end  
                            end               
                        end        
                    else
                        w = T(3);
                        if w > T(4)
                            if w > T(5)
                                if w > T(6)
                                    Table2 = T(3);
                                else
                                    Table2 = T(6);
                                end                
                            else
                                w = T(5);
                                if w > T(6)
                                    Table2 = T(5);
                                else
                                    Table2 = T(6);
                                end  
                            end            
                        else
                            w = T(4);
                            if w > T(5)
                                if w > T(6)
                                    Table2 = T(4);
                                else
                                    Table2 = T(6);
                                end                
                            else
                                w = T(5);
                                if w > T(6)
                                    Table2 = T(5);
                                else
                                    Table2 = T(6);
                                end  
                            end               
                        end 
                    end   
                else
                    w = T(2);
                    if w > T(3)
                        if w > T(4)
                            if w > T(5)
                                if w > T(6)
                                    Table2 = T(2);
                                else
                                    Table2 = T(6);
                                end                
                            else
                                w = T(5);
                                if w > T(6)
                                    Table2 = T(5);
                                else
                                    Table2 = T(6);
                                end  
                            end            
                        else
                            w = T(4);
                            if w > T(5)
                                if w > T(6)
                                    Table2 = T(4);
                                else
                                    Table2 = T(6);
                                end                
                            else
                                w = T(5);
                                if w > T(6)
                                    Table2 = T(5);
                                else
                                    Table2 = T(6);
                                end  
                            end               
                        end        
                    else
                        w = T(3);
                        if w > T(4)
                            if w > T(5)
                                if w > T(6)
                                    Table2 = T(3);
                                else
                                    Table2 = T(6);
                                end                
                            else
                                w = T(5);
                                if w > T(6)
                                    Table2 = T(5);
                                else
                                    Table2 = T(6);
                                end  
                            end            
                        else
                            w = T(4);
                            if w > T(5)
                                if w > T(6)
                                    Table2 = T(4);
                                else
                                    Table2 = T(6);
                                end                
                            else
                                w = length(T(5));
                                if w > T(6)
                                    Table2 = T(5);
                                else
                                    Table2 = T(6);
                                end  
                            end               
                        end 
                    end     
                end

                if Table2 == length(Panier1)
                    Table2 = Panier1;
                elseif Table2 == length(Panier2)
                    Table2 = Panier2; 
                elseif Table2 == length(Panier3)
                    Table2 = Panier3;
                elseif Table2 == length(Panier4)
                    Table2 = Panier4; 
                elseif Table2 == length(Panier5)
                    Table2 = Panier5; 
                elseif Table2 == length(Panier6)
                    Table2 = Panier6;
                elseif Table2 == length(Panier7)
                    Table2 = Panier6;
                end

                % ------------------------Erase the false baskets-------------------------

                % Basket1.
                if length(Panier1) ~= length(Table1) && length(Panier1) ~= length(Table2)
                    Basket1 = Panier1; 
                    Panier1 = zeros(length(Table1));
                elseif length(Panier2) ~= length(Table1) && length(Panier2) ~= length(Table2)
                    Basket1 = Panier2; 
                    Panier2 = zeros(length(Table1));
                elseif length(Panier3) ~= length(Table1) && length(Panier3) ~= length(Table2)
                    Basket1 = Panier3; 
                    Panier3 = zeros(length(Table1));
                elseif length(Panier4) ~= length(Table1) && length(Panier4) ~= length(Table2)
                    Basket1 = Panier4;
                    Panier4 = zeros(length(Table1));
                elseif length(Panier5) ~= length(Table1) && length(Panier5) ~= length(Table2)
                    Basket1 = Panier5; 
                    Panier5 = zeros(length(Table1));
                elseif length(Panier6) ~= length(Table1) && length(Panier6) ~= length(Table2)
                    Basket1 = Panier6; 
                    Panier6 = zeros(length(Table1));
                elseif length(Panier7) ~= length(Table1) && length(Panier7) ~= length(Table2)
                    Basket1 = Panier7; 
                    Panier7 = zeros(length(Table1));
                end

                % Basket2.
                if length(Panier1) ~= length(Table1) && length(Panier1) ~= length(Table2)
                    Basket2 = Panier1; 
                    Panier1 = zeros(length(Table1));
                elseif length(Panier2) ~= length(Table1) && length(Panier2) ~= length(Table2)
                    Basket2 = Panier2; 
                    Panier2 = zeros(length(Table1));
                elseif length(Panier3) ~= length(Table1) && length(Panier3) ~= length(Table2)
                    Basket2 = Panier3; 
                    Panier3 = zeros(length(Table1));
                elseif length(Panier4) ~= length(Table1) && length(Panier4) ~= length(Table2)
                    Basket2 = Panier4;
                    Panier4 = zeros(length(Table1));
                elseif length(Panier5) ~= length(Table1) && length(Panier5) ~= length(Table2)
                    Basket2 = Panier5; 
                    Panier5 = zeros(length(Table1));
                elseif length(Panier6) ~= length(Table1) && length(Panier6) ~= length(Table2)
                    Basket2 = Panier6; 
                    Panier6 = zeros(length(Table1));
                elseif length(Panier7) ~= length(Table1) && length(Panier7) ~= length(Table2)
                    Basket2 = Panier7; 
                    Panier7 = zeros(length(Table1));
                end

                % Basket3.
                if length(Panier1) ~= length(Table1) && length(Panier1) ~= length(Table2)
                    Basket3 = Panier1; 
                    Panier1 = zeros(length(Table1));
                elseif length(Panier2) ~= length(Table1) && length(Panier2) ~= length(Table2)
                    Basket3 = Panier2; 
                    Panier2 = zeros(length(Table1));
                elseif length(Panier3) ~= length(Table1) && length(Panier3) ~= length(Table2)
                    Basket3 = Panier3; 
                    Panier3 = zeros(length(Table1));
                elseif length(Panier4) ~= length(Table1) && length(Panier4) ~= length(Table2)
                    Basket3 = Panier4;
                    Panier4 = zeros(length(Table1));
                elseif length(Panier5) ~= length(Table1) && length(Panier5) ~= length(Table2)
                    Basket3 = Panier5; 
                    Panier5 = zeros(length(Table1));
                elseif length(Panier6) ~= length(Table1) && length(Panier6) ~= length(Table2)
                    Basket3 = Panier6; 
                    Panier6 = zeros(length(Table1));
                elseif length(Panier7) ~= length(Table1) && length(Panier7) ~= length(Table2)
                    Basket3 = Panier7; 
                    Panier7 = zeros(length(Table1));
                end

                % Basket4.
                if length(Panier1) ~= length(Table1) && length(Panier1) ~= length(Table2)
                    Basket4 = Panier1; 
                    Panier1 = zeros(length(Table1));
                elseif length(Panier2) ~= length(Table1) && length(Panier2) ~= length(Table2)
                    Basket4 = Panier2; 
                    Panier2 = zeros(length(Table1));
                elseif length(Panier3) ~= length(Table1) && length(Panier3) ~= length(Table2)
                    Basket4 = Panier3; 
                    Panier3 = zeros(length(Table1));
                elseif length(Panier4) ~= length(Table1) && length(Panier4) ~= length(Table2)
                    Basket4 = Panier4;
                    Panier4 = zeros(length(Table1));
                elseif length(Panier5) ~= length(Table1) && length(Panier5) ~= length(Table2)
                    Basket4 = Panier5; 
                    Panier5 = zeros(length(Table1));
                elseif length(Panier6) ~= length(Table1) && length(Panier6) ~= length(Table2)
                    Basket4 = Panier6; 
                    Panier6 = zeros(length(Table1));
                elseif length(Panier7) ~= length(Table1) && length(Panier7) ~= length(Table2)
                    Basket4 = Panier7; 
                    Panier7 = zeros(length(Table1));
                end

                % Basket5.
                if length(Panier1) ~= length(Table1) && length(Panier1) ~= length(Table2)
                    Basket5 = Panier1; 
                    Panier1 = zeros(length(Table1));
                elseif length(Panier2) ~= length(Table1) && length(Panier2) ~= length(Table2)
                    Basket5 = Panier2; 
                    Panier2 = zeros(length(Table1));
                elseif length(Panier3) ~= length(Table1) && length(Panier3) ~= length(Table2)
                    Basket5 = Panier3; 
                    Panier3 = zeros(length(Table1));
                elseif length(Panier4) ~= length(Table1) && length(Panier4) ~= length(Table2)
                    Basket5 = Panier4;
                    Panier4 = zeros(length(Table1));
                elseif length(Panier5) ~= length(Table1) && length(Panier5) ~= length(Table2)
                    Basket5 = Panier5; 
                    Panier5 = zeros(length(Table1));
                elseif length(Panier6) ~= length(Table1) && length(Panier6) ~= length(Table2)
                    Basket5 = Panier6; 
                    Panier6 = zeros(length(Table1));
                elseif length(Panier7) ~= length(Table1) && length(Panier7) ~= length(Table2)
                    Basket5 = Panier7; 
                    Panier7 = zeros(length(Table1));
                end

                Lim = [min(Basket1(:,1)) max(Basket1(:,1)) min(Basket1(:,2)) max(Basket1(:,2));
                    min(Basket2(:,1)) max(Basket2(:,1)) min(Basket2(:,2)) max(Basket2(:,2));
                    min(Basket4(:,1)) max(Basket4(:,1)) min(Basket4(:,2)) max(Basket4(:,2));
                    min(Basket5(:,1)) max(Basket5(:,1)) min(Basket5(:,2)) max(Basket5(:,2));
                    min(Basket3(:,1)) max(Basket3(:,1)) min(Basket3(:,2)) max(Basket3(:,2))];  

                totalMap = totalMap2;

                % prepare value to be saved
                tableGoal = zeros(2);
                tableGoal(1,1) = (min(Table1(:,1)) + max(Table1(:,1)))/2;
                tableGoal(1,2) = (min(Table1(:,2)) + max(Table1(:,2)))/2;
                tableGoal(2,1) = (min(Table2(:,1)) + max(Table2(:,1)))/2;
                tableGoal(2,2) = (min(Table2(:,2)) + max(Table2(:,2)))/2;
                tableGoal = round(tableGoal);

                basketCenter = zeros(5,2);
                basketGoal = zeros(5,2);
                
                for i = 1:5
                    basketCenter(i,1) = (Lim(i,1) + Lim(i,2))/2;
                    basketCenter(i,2) = (Lim(i,3) + Lim(i,4))/2;

                    if (totalMap(Lim(i,1)-1, Lim(i,3)-1) == 0)
                        basketGoal(i,:) = [Lim(i,1)-1 Lim(i,3)-1];

                    elseif (totalMap(Lim(i,2)+1, Lim(i,3)-1) == 0)
                        basketGoal(i,:) = [Lim(i,2)+1 Lim(i,3)-1];

                    elseif (totalMap(Lim(i,1)-1, Lim(i,4)+1) == 0)
                        basketGoal(i,:) = [Lim(i,1)-1 Lim(i,4)+1];

                    elseif (totalMap(Lim(i,2)+1, Lim(i,4)+1) == 0)
                        basketGoal(i,:) = [Lim(i,2)+1 Lim(i,4)+1];

                    end
                end

                basketCenter = basketCenter;
                basketGoal = round(basketGoal);

                save('basketGoal.mat', 'basketGoal');
                save('basketCenter.mat', 'basketCenter');
                save('tableGoal.mat','tableGoal');
                pause(1)

                fsm =  'begin_travel';
                interstep = 'vision';

            elseif strcmp(fsm, 'begin_travel') 
                
                fprintf('Go to table: %d\n', counter);

                NewGoal = basketGoal(counter,:);

                step = 'travel';
                fsm = 'newdestination';
                interfsm = 'table rotate';

            elseif strcmp(fsm, 'table rotate')
                NewAngl = atan(((resol*youbotPos(1)-minMap(1)+1)-basketCenter(counter,1))/...
                    (basketCenter(counter,2)-(resol*youbotPos(2)-minMap(2)+1))) - pi/2 ;
                if (basketCenter(counter,2)-(resol*youbotPos(2)-minMap(2)+1)) > 0
                    NewAngl = NewAngl + pi;
                end

                step = 'travel';
                fsm = 'rotate';
                interfsm = 'table_end';
                interval = 'only_rotate';

            elseif strcmp(fsm, 'table_end')

                % get rgb camera position
                [res, rgbdPos] = vrep.simxGetObjectPosition(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);

                NewAngl = atan(((resol*rgbdPos(1)-minMap(1)+1)- basketCenter(counter,1))/...
                    (basketCenter(counter,2)-(resol*rgbdPos(2)-minMap(2)+1))) - pi/2 ;
                if (basketCenter(counter,2)-(resol*rgbdPos(2)-minMap(2)+1)) > 0
                    NewAngl = NewAngl + pi;
                end
                vrep.simxSetObjectOrientation(id, h.rgbdCasing, -1,[0 0 NewAngl], vrep.simx_opmode_oneshot);
                pause(1);

                % get rgb camera angle
                [res, rgbdEuler] = vrep.simxGetObjectOrientation(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
                
                res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/3, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
                
                res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res); % Check the return value from the previous V-REP call (res) and exit in case of error. 

                % capture images
                fprintf('Capturing image...\n');
                [res, resolution, images{counter}] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
                fprintf('Captured %i pixels (%i x %i).\n', resolution(1) * resolution(2), resolution(1), resolution(2));

                % show the image. 
                figure; 
                imshow(images{counter});
                drawnow;

                baseFileName = sprintf('Image #%d.png', counter);
                fullFileName = fullfile(fullfile(pwd() ,'testPictures'), baseFileName);
                imwrite(images{counter}, fullFileName);
                
                if counter >= 5
                    fsm = 'identifiedimage';
                else
                    counter = counter + 1;
                    fsm = 'begin_travel';
                end

            elseif strcmp(fsm, 'identifiedimage')


                %reco image

                fsm = 'start';

                if fullcode
                    step = 'manipulation';
                else
                    step = 'finished';
                end
                fprintf('Switching to step: %s\n', step);

            else
                step = 'finished';
                error('Unknown state on vision %s.', fsm);
            end

        
            
        elseif strcmp(step, 'manipulation')
            
            if strcmp(fsm, 'start')
                
                interstep = 'manipulation';
                
                if ~fullcode  
                    % save map and boundary value to avoid to run navigation that
                    % is very long (~8min)
                    Map = load('totalmap.mat');
                    totalMap = cell2mat(struct2cell(Map));
                    Map = load('minmap.mat');
                    minMap = cell2mat(struct2cell(Map));
                    Map = load('maxmap.mat');
                    maxMap = cell2mat(struct2cell(Map));
                    Map = load('tableGoal.mat');
                    tableGoal = cell2mat(struct2cell(Map));
                    
                end
                %turn off the hokuyo captor
                res = vrep.simxSetIntegerSignal(h.id, 'handle_xy_sensor', 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res);             
                
                figure;
                imagesc(totalMap)
                axis equal
               
                pts_tableTot1 = [];
                pts_blockTot1 = [];
                pts_tableTot2 = [];
                pts_blockTot2 = [];
                
                % to evaluate previously
                centerTable1 = [tableGoal(1,1), tableGoal(1,2)];
                centerTable2 = [tableGoal(2,1), tableGoal(2,2)];
                
                centerTable =  centerTable1;
                
                % find visible point around the table 1
                destTable1 = zeros(4,2);
                indexTable1 = 1;
                for i = 1:5
                    if totalMap(centerTable1(1)+i, centerTable1(2)) == 0
                        destTable1(4,:) = [centerTable1(1)+i, centerTable1(2)];
                        break;
                    end
                end
                for i = 1:5
                    if totalMap(centerTable1(1)-i, centerTable1(2)) == 0
                        destTable1(2,:) = [centerTable1(1)-i, centerTable1(2)];
                        break;
                    end
                end
                for i = 1:5
                    if totalMap(centerTable1(1), centerTable1(2)+i) == 0
                        destTable1(1,:) = [centerTable1(1), centerTable1(2)+i];
                        break;
                    end
                end
                for i = 1:5
                    if totalMap(centerTable1(1), centerTable1(2)-i) == 0
                        destTable1(3,:) = [centerTable1(1), centerTable1(2)-i];
                        break;
                    end
                end
                destTable2 = zeros(4,2);
                indexTable2 = 1;
                % find visible point around the table 2 
                for i = 1:5
                    if totalMap(centerTable2(1)+i, centerTable2(2)) == 0
                        destTable2(4,:) = [centerTable2(1)+i, centerTable2(2)];
                        break;
                    end
                end
                for i = 1:5
                    if totalMap(centerTable2(1)-i, centerTable2(2)) == 0
                        destTable2(2,:) = [centerTable2(1)-i, centerTable2(2)];
                        break;
                    end
                end 
                for i = 1:5
                    if totalMap(centerTable2(1), centerTable2(2)+i) == 0
                        destTable2(1,:) = [centerTable2(1), centerTable2(2)+i];
                        break;
                    end
                end
                for i = 1:5
                    if totalMap(centerTable2(1), centerTable2(2)-i) == 0
                        destTable2(3,:) = [centerTable2(1), centerTable2(2)-i];
                        break;
                    end
                end
                
                fsm = 'move_center';
                interfsm = 'move_center';
                
            elseif strcmp(fsm, 'move_center') 
                
                if indexTable1 == 5
                    centerTable =  centerTable2;
                    % visit table 2 
                    for i = indexTable2:4
                        indexTable2 = indexTable2 +1;
                        if(destTable2(i,1) ~= 0)||(destTable2(i,2) ~= 0)
                            NewGoal(1) = destTable2(i,1);
                            NewGoal(2) = destTable2(i,2); 
                            break;
                        end
                    end
                else
                    % visit table 1 
                    for i = indexTable1:4
                        indexTable1 = indexTable1 +1;
                        if(destTable1(i,1) ~= 0)||(destTable1(i,2) ~= 0)
                            NewGoal(1) = destTable1(i,1);
                            NewGoal(2) = destTable1(i,2);
                            break;
                        end
                    end
                end
                step = 'travel';
                fsm = 'newdestination';
                interfsm = 'move_center2'; 
                
                
            elseif strcmp(fsm, 'move_center2')
                NewAngl = atan(((resol*youbotPos(1)-minMap(1)+1)-centerTable(1))/...
                    (centerTable(2)-(resol*youbotPos(2)-minMap(2)+1))) - pi/2 ;
                if (centerTable(2)-(resol*youbotPos(2)-minMap(2)+1)) > 0
                    NewAngl = NewAngl + pi;
                end
        
                step = 'travel';
                fsm = 'rotate';
                interfsm = 'move_center3';
                interval = 'only_rotate';
                
              
            elseif strcmp(fsm, 'move_center3')
                % get rgb camera position
                [res, rgbdPos] = vrep.simxGetObjectPosition(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
                
                NewAngl = atan(((resol*rgbdPos(1)-minMap(1)+1)-centerTable(1))/...
                    (centerTable(2)-(resol*rgbdPos(2)-minMap(2)+1))) - pi/2 ;
                if (centerTable(2)-(resol*rgbdPos(2)-minMap(2)+1)) > 0
                    NewAngl = NewAngl + pi;
                end
                vrep.simxSetObjectOrientation(id, h.rgbdCasing, -1,[0 0 NewAngl], vrep.simx_opmode_oneshot);
                 
                % get rgb camera angle
                [res, rgbdEuler] = vrep.simxGetObjectOrientation(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
                
                 % large view to identify block
                res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/4, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
                 
                % take a 3D image
                res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
                % and store this image in pts
                pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
                % Each column of pts has [x;y;z;distancetosensor]. However, plot3 does not have the same frame of reference as 
                % the output data. To get a correct plot, you should invert the y and z dimensions. 
                
                %we only keep points within 1.2 meter, to focus on the table. 
                pts = pts(1:3, pts(4, :) < 1.2);
 
                % transformation in the right pose
                pts = [pts(3,:);pts(1,:);pts(2,:)];
                trf = transl(rgbdPos) * trotx(rgbdEuler(1)) * troty(rgbdEuler(2)) * trotz(rgbdEuler(3));
                pts = homtrans(trf, pts);
                
                % the height of the table is 185 mm
                % we conserv point with a height between 50 and 100 mm by security
                % to identify the table and not see the robot with the captor
                pts_table = pts(1:3,pts(3,:) < 0.10);
                pts_table = pts_table(1:2,pts_table(3,:) > 0.05);
                % we conserv point with a height greater than 187 mm by security
                % to identify the blocks on the table
                pts_block = pts(1:3,pts(3,:) > 0.187 );
                
                 % small view to identify block
                res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/7, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
                 
                % take a 3D image
                res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
                % and store this image in pts
                pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
                % Each column of pts has [x;y;z;distancetosensor]. However, plot3 does not have the same frame of reference as 
                % the output data. To get a correct plot, you should invert the y and z dimensions. 
                
                %we only keep points within 1.2 meter, to focus on the table. 
                pts = pts(1:3, pts(4, :) < 1.2);
 
                % transformation in the right pose
                pts = [pts(3,:);pts(1,:);pts(2,:)];
                pts = homtrans(trf, pts);
                
                % we conserv point with a height greater than 187 mm by security
                % to identify the blocks on the table
                pts_block2 = pts(1:3,pts(3,:) > 0.187 );
                
                fsm = 'move_center';
                
                if (centerTable(1) == centerTable1(1))&&(centerTable(2) == centerTable1(2))
                    % save points of the table 1 in a global matrix and remove the
                    % multiple points
                    pts_tableTot1 = unique([pts_tableTot1 ;transpose(round(250*pts_table)/250)], 'rows');
                    pts_blockTot1 = unique([pts_blockTot1 ;transpose(round(250*pts_block)/250);...
                        transpose(round(250*pts_block2)/250)], 'rows');
                     
                else
                    % save points of the table 1 in a global matrix and remove the
                    % multiple points
                    pts_tableTot2 = unique([pts_tableTot2 ;transpose(round(250*pts_table)/250)], 'rows');
                    pts_blockTot2 = unique([pts_blockTot2 ;transpose(round(250*pts_block)/250);...
                        transpose(round(250*pts_block2)/250)], 'rows');
                    if indexTable2 == 5 
                        fsm = 'finished';
                    end
                end
                
            elseif strcmp(fsm, 'finished')
               
                % plot block on the table 1
                figure;
                plot3(pts_blockTot1(:, 1), pts_blockTot1(:, 2), pts_blockTot1(:, 3), '*');
                axis equal;
                %plot the table 1
                figure;
                plot(pts_tableTot1(:,1), pts_tableTot1(:,2), '*')
               
                % plot block on the table 2
                figure;
                plot3(pts_blockTot2(:, 1), pts_blockTot2(:, 2), pts_blockTot2(:, 3), '*');
                axis equal;
                %plot the table 2
                figure;
                plot(pts_tableTot2(:,1), pts_tableTot2(:,2), '*')
     
                if fullcode
                    step = 'manipulation2';
                else
                    step = 'finished';
                     % save the matrix for image analysis
                    save('pts_blocktot1.mat', 'pts_blockTot1');
                    save('pts_tabletot1.mat', 'pts_tableTot1');                   
                    save('pts_blocktot2.mat', 'pts_blockTot2');
                    save('pts_tabletot2.mat', 'pts_tableTot2');                   
                end
                fprintf('Switching to step: %s\n', step);
            end
            
     
        elseif strcmp(step, 'manipulation2')
            interstep = 'manipulation2';
            if strcmp(fsm, 'start')
                
                if ~fullcode  
                    % save map and boundary value to avoid to run navigation that
                    % is very long (~8min)
                    Map = load('totalmap.mat');
                    totalMap = cell2mat(struct2cell(Map));
                    Map = load('minmap.mat');
                    minMap = cell2mat(struct2cell(Map));
                    Map = load('maxmap.mat');
                    maxMap = cell2mat(struct2cell(Map));
%                     Map = load('basketId.mat');
%                     basketId = cell2mat(struct2cell(Map));
                    Map = load('basketGoal.mat');
                    basketGoal = cell2mat(struct2cell(Map));
                    Map = load('basketCenter.mat');
                    basketCenter = cell2mat(struct2cell(Map));
                    % extract the data to only work on image
                    Map = load('pts_blocktot1.mat');
                    pts_blockTot1 = cell2mat(struct2cell(Map));
                    Map = load('pts_tabletot1.mat');
                    pts_tableTot1 = cell2mat(struct2cell(Map));

                    Map = load('pts_blocktot2.mat');
                    pts_blockTot2 = cell2mat(struct2cell(Map));
                    Map = load('pts_tabletot2.mat');
                    pts_tableTot2 = cell2mat(struct2cell(Map));
                    
                    load('instructions.mat');
                    load('pictureBasket.mat');
                
                end
                %turn off the hokuyo captor
                res = vrep.simxSetIntegerSignal(h.id, 'handle_xy_sensor', 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res); 
                 
                real_centerTable1 = [(max(pts_tableTot1(:, 1)) + min(pts_tableTot1(:, 1)))/2 ...
                    (max(pts_tableTot1(:, 2))+min(pts_tableTot1(:, 2)))/2];
                real_centerTable2 = [(max(pts_tableTot2(:, 1)) + min(pts_tableTot2(:, 1)))/2 ...
                    (max(pts_tableTot2(:, 2))+min(pts_tableTot2(:, 2)))/2];
                
                % find center of blocks thanks to kmeans
                % 50 iterates to have the expected centers is a good
                % trade-off
                weight1 = Inf;
                for i = 1 : 50                
                    [idBlock,centerBlock, weight] = kmeans(pts_blockTot1, 5);
                    if(mean(weight) < weight1)               
                        idBlock1 = idBlock;
                        centerBlock1 = centerBlock;
                        weight1 = mean(weight);
                    end
                end
                
                figure;
                plot3(pts_blockTot1(:,1), pts_blockTot1(:,2),pts_blockTot1(:,3),'.',...
                     centerBlock1(:,1), centerBlock1(:,2), centerBlock1(:,3),'*')
                
                                
                weight2 = Inf;
                for i = 1 : 50                
                    [idBlock,centerBlock, weight] = kmeans(pts_blockTot2, 5);
                    if(mean(weight) < weight2)               
                        idBlock2 = idBlock;
                        centerBlock2 = centerBlock;
                        weight2 = mean(weight);
                    end
                end
                
                 figure;
                 plot3(pts_blockTot2(:,1), pts_blockTot2(:,2),pts_blockTot2(:,3),'.',...
                     centerBlock2(:,1), centerBlock2(:,2), centerBlock2(:,3),'*')
                              
                if max(bsxfun(@hypot, centerBlock1(:, 1)- mean(centerBlock1(:, 1)), ...
                    centerBlock1(:, 2)- mean(centerBlock1(:, 2)))) > ...
                    max(bsxfun(@hypot, centerBlock2(:, 1)- mean(centerBlock2(:, 1)), ...
                    centerBlock2(:, 2)- mean(centerBlock2(:, 2))))
                    % easy table is table 1 because more space between block
                    easyTable  = 1;
                    centerBlocks = centerBlock1;
                    idBlocks = idBlock1;
                    centerBlockHard =[ mean(centerBlock2(:, 1)) mean(centerBlock2(:, 2))];
                    
                else 
                     
                    easyTable  = 2;            
                    % invert tables
                    inter = real_centerTable1;
                    real_centerTable1 = real_centerTable2;
                    real_centerTable2 = inter;
                    
                    inter = pts_blockTot1;
                    pts_blockTot1 = pts_blockTot2;
                    pts_blockTot2 = inter;
                    
                    inter = pts_tableTot1;
                    pts_tableTot1 = pts_tableTot2;
                    pts_tableTot2 = inter;
                    
                    centerBlocks = centerBlock2;
                    idBlocks = idBlock2;
                    centerBlockHard =[ mean(centerBlock1(:, 1)) mean(centerBlock1(:, 2))];
                    
                end
                 
                index_easyTable = 1;
                index_hardTable = 1;
             
                fsm = 'moveTable';
                
            elseif strcmp(fsm, 'moveTable')
                
                if index_easyTable == 6
                    %hard table not us e in practice
                    if index_hardTable == 6
                        step = 'finished';
                        break;
                    end
                    real_centerTable = real_centerTable2;
                    blockpoint = [centerBlockHard 0];
                    index_hardTable = index_hardTable + 1;
                    
                    
                else
                    % easy table
                    
                    % for the test phase
                    if index_easyTable == 6
                        step = 'finished';
                        break;
                    end
                    
                    blockpoint = centerBlocks(index_easyTable, :);
                    real_centerTable = real_centerTable1;
                    
                    index_easyTable = index_easyTable + 1;
                end

               AnglDestBlock = atan((real_centerTable(2) - blockpoint(2))/...
                    (real_centerTable(1) - blockpoint(1))) ;
                if (real_centerTable(1) - blockpoint(1)) > 0
                    AnglDestBlock = AnglDestBlock + pi;
                end
                
                destBlock = [(round(resol*(real_centerTable(1) + 0.7 * cos(AnglDestBlock)))- minMap(1)+1)...
                    (round(resol*(real_centerTable(2) + 0.7 * sin(AnglDestBlock)))- minMap(2)+1)];
                
                if(totalMap(destBlock(1), destBlock(2))~= 0)
                    destBlock = [(round(resol*(real_centerTable(1) + 0.8 * cos(AnglDestBlock)))- minMap(1)+1)...
                    (round(resol*(real_centerTable(2) + 0.8 * sin(AnglDestBlock)))- minMap(2)+1)];
                end
                
                if(totalMap(destBlock(1), destBlock(2))~= 0)
                    destBlock = [(round(resol*(real_centerTable(1) + 0.9 * cos(AnglDestBlock)))- minMap(1)+1)...
                    (round(resol*(real_centerTable(2) + 0.9 * sin(AnglDestBlock)))- minMap(2)+1)];
                end
                
                if(totalMap(destBlock(1), destBlock(2))~= 0)
                    destBlock = [(round(resol*(real_centerTable(1) + 1 * cos(AnglDestBlock)))- minMap(1)+1)...
                    (round(resol*(real_centerTable(2) + 1 * sin(AnglDestBlock)))- minMap(2)+1)];
                end
                
                if(totalMap(destBlock(1), destBlock(2))~= 0)
                    destBlock = [(round(resol*(real_centerTable(1) + 1.1 * cos(AnglDestBlock)))- minMap(1)+1)...
                    (round(resol*(real_centerTable(2) + 1.1 * sin(AnglDestBlock)))- minMap(2)+1)];
                end
                
                if(totalMap(destBlock(1), destBlock(2))~= 0)
                    destBlock = [(round(resol*(real_centerTable(1) + 1.2 * cos(AnglDestBlock)))- minMap(1)+1)...
                    (round(resol*(real_centerTable(2) + 1.2 * sin(AnglDestBlock)))- minMap(2)+1)];
                end
                
                NewGoal = destBlock;
                
                step = 'travel';
                fsm = 'newdestination';
                interfsm = 'moveTable2';
                        
            elseif strcmp(fsm, 'moveTable2')
                % rotation to be paralel to the surface of the table
                NewAngl = atan((youbotPos(1)- real_centerTable(1))/...
                    (real_centerTable(2)-youbotPos(2))) - pi/2 ;
                if (real_centerTable(2)-youbotPos(2)) > 0
                    NewAngl = NewAngl + pi;
                end
                      
                step = 'travel';
                fsm = 'rotate';
                interfsm = 'moveTable3';
                interval = 'only_rotate';
                
            elseif strcmp(fsm, 'moveTable3')
                     
                blockFarPoint = [youbotPos(1) youbotPos(2)];       
                AnglDestBlock = atan((real_centerTable(2) - blockFarPoint(2))/...
                    (real_centerTable(1) - blockFarPoint(1))) ;
                if (real_centerTable(1) - blockFarPoint(1)) > 0
                    AnglDestBlock = AnglDestBlock + pi;
                end
                
                blockClosePoint = [(real_centerTable(1) + 0.67 * cos(AnglDestBlock))...
                    (real_centerTable(2) + 0.67 * sin(AnglDestBlock))];
                   
                initialDistSlide = sqrt((blockFarPoint(1) -  blockClosePoint(1))^2 ...
                        +(blockFarPoint(2) -  blockClosePoint(2))^2);
                interPosSlide = blockFarPoint;
                prevPosSlide = blockFarPoint;
                                          
                fsm = 'slide';
                interfsm  = 'takepicture';
                  
            elseif strcmp(fsm, 'slide')
                DistPrev = sqrt((interPosSlide(1) - youbotPos(1))^2 +...
                (interPosSlide(2) - youbotPos(2))^2);
            
                % If the distance is too short, we can't go to the maximum speed.
                if DistPrev <= initialDistSlide   
                    if initialDistSlide < 2*maxDist
                        middleDist = initialDistSlide/2;

                        % Acceleration.
                        rightVel = -maxSpeed/maxDist * DistPrev;

                        % Decceleration.
                        if DistPrev > middleDist
                            rightVel = -maxSpeed/maxDist * (maxDist-DistPrev);
                        end

                    else
                        if DistPrev < maxDist
                            rightVel = -maxSpeed/maxDist * DistPrev;
                        elseif DistPrev < initialDistSlide - maxDist
                            rightVel = -maxSpeed;
                        else
                            rightVel = -maxSpeed/maxDist * (maxDist-DistPrev);
                        end
                    end
                    if (DistPrev - initialDistSlide) < 2*maxDist
                        middleDist = (DistPrev - initialDistSlide)/2;
                        % Acceleration
                        rightVel = maxSpeed/maxDist * (maxDist - (DistPrev - initialDistSlide));

                        % Decceleration
                        if (DistPrev - initialDistSlide) < middleDist
                            rightVel = maxSpeed/maxDist * (DistPrev - initialDistSlide);
                        end 
                    else                        
                        if (DistPrev - initialDistSlide) < maxDist
                            rightVel = maxSpeed/maxDist * (DistPrev - initialDistSlide);
                        elseif (DistPrev - initialDistSlide) < (DistPrev - initialDistSlide) - maxDist
                            rightVel = maxSpeed;
                        else
                            rightVel = maxSpeed/maxDist * (maxDist - (DistPrev - initialDistSlide));
                        end
                    end
                end
                
                 % If the goal is reached stop the robot.
                if (abs(initialDistSlide - DistPrev) < 0.01) && (sqrt((prevPosSlide(1)-youbotPos(1))^2 + ...
                    (prevPosSlide(2)-youbotPos(2))^2) < 0.005)
                
                    rightVel = 0;
                    step = interstep;
                    fsm = interfsm;
                end
                
                prevPosSlide = [youbotPos(1) youbotPos(2)];
                
                if (interPosSlide(1) ==  blockFarPoint(1))&&(interPosSlide(2) ==  blockFarPoint(2))
                    rightVel = - rightVel;
                end

                
            elseif strcmp(fsm, 'takepicture')    
                % get rgb camera position
                [res, rgbdPos] = vrep.simxGetObjectPosition(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
                
                NewAngl = atan((rgbdPos(1)- blockpoint(1))/...
                    (blockpoint(2)-rgbdPos(2))) - pi/2 ;
                if (blockpoint(2)-rgbdPos(2)) > 0
                    NewAngl = NewAngl + pi;
                end
                
                vrep.simxSetObjectOrientation(id, h.rgbdCasing, -1,[0 0 NewAngl], vrep.simx_opmode_oneshot);
                 
                % get rgb camera angle
                [res, rgbdEuler] = vrep.simxGetObjectOrientation(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
                
                 % small view to identify block
                res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/8, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
                 
                % take a 3D image
                res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
                % and store this image in pts
                pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
                % Each column of pts has [x;y;z;distancetosensor]. However, plot3 does not have the same frame of reference as 
                % the output data. To get a correct plot, you should invert the y and z dimensions. 
                
                %we only keep points within 0.8 meter, to focus on the table. 
                pts = pts(1:3, pts(4, :) < 0.8);
                
                % transformation in the right pose
                pts = [pts(3,:);pts(1,:);pts(2,:)];
                trf = transl(rgbdPos) * trotx(rgbdEuler(1)) * troty(rgbdEuler(2)) * trotz(rgbdEuler(3));
                pts_block = homtrans(trf, pts);
                
                % remove the table on the image
                pts_block = transpose(pts_block(:,pts_block(3,:) > 0.187));
                % conserv only portion around the center point
                pts_block = unique(pts_block((pts_block(:,1)>(blockpoint(1)-0.06)),1:2), 'rows');
                pts_block = pts_block((pts_block(:,1)<(blockpoint(1)+0.06)),:);
                pts_block = pts_block((pts_block(:,2)>(blockpoint(2)-0.06)),:);
                pts_block = pts_block((pts_block(:,2)<(blockpoint(2)+0.06)),:);
                  
                %identify form
                lengthsampleform = 10;
                
                if (size(pts_block, 1)-lengthsampleform -1)< lengthsampleform
                    lengthsampleform = 5;
                end
               
                interform = randi([1 (size(pts_block, 1)-lengthsampleform -1)],[size(pts_block, 1) 1]);

                anglform = zeros(size(pts_block, 1), lengthsampleform);
                for j = 1:lengthsampleform
                    anglform(:, j) = atan((pts_block(interform+j,1)- pts_block(interform+j+1,1))./...
                                            (pts_block(interform+j,2)- pts_block(interform+j+1,2)));
                end
                
                stdform = min(var(anglform,0,2)) 
                if stdform < 0.001
                    formBlock = 'cylinder'
                else
                    formBlock = 'box'
                end
            
                %identify color
                res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);

                fprintf('Capturing image...\n');
                [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
                fprintf('Captured %i pixels (%i x %i).\n', resolution(1) * resolution(2), resolution(1), resolution(2));

                % Finally, show the image. 
                figure;
                imshow(image);
                drawnow;


                intercolor = image(resolution(1)/2 -5:resolution(1)/2 +5, resolution(2)/2 -5:resolution(2)/2 +5,:);
                colorBlock = [mean(mean(intercolor(:,:,1))) mean(mean(intercolor(:,:,2))) mean(mean(intercolor(:,:,3)))];
                colorValue = 0;
                if(colorBlock(1) > 150)
                    colorValue = colorValue + 1;
                end
                if(colorBlock(2) > 150)
                    colorValue = colorValue + 2;
                end
                if(colorBlock(3) > 150)
                    colorValue = colorValue + 4;
                end

                switch colorValue        
                    case 1
                        colorName = 'red'
                    case 2
                        colorName = 'green'
                    case 3
                        colorName = 'yellow'
                    case 4
                        colorName = 'blue'
                    case 5
                        colorName = 'purple'
                    case 7
                        colorName = 'white'
                end

                         
                fsm = 'move1gras';
           
            elseif strcmp(fsm, 'move1gras')
                % get rgb position
                [res, rgbdPos] = vrep.simxGetObjectPosition(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
                    
                gripPos = [0 0.166 0] ;
                trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
                gripPos = homtrans(trf, transpose(gripPos)); 
                
                indexBlock = find(bsxfun(@hypot, pts_block(:,1)- gripPos(1), pts_block(:,2)-gripPos(2))==...
                    min(bsxfun(@hypot, pts_block(:,1)-gripPos(1), pts_block(:,2)-gripPos(2))),1,'first');
                
                figure;
                plot(pts_block(:,1), pts_block(:,2), '*',rgbdPos(1), rgbdPos(2), '.', gripPos(1), gripPos(2), '+',...
                     blockpoint(1),blockpoint(2),'*' , youbotPos(1),  youbotPos(2), '^', pts_block(indexBlock,1), pts_block(indexBlock,2), '*' );
                            
                blockpoint(1) = (2*blockpoint(1) + pts_block(indexBlock, 1))/3;
                blockpoint(2) = (2*blockpoint(2) + pts_block(indexBlock, 2))/3;
            
                 % rotation to the block on the table
                 NewAngl = atan((gripPos(1)- blockpoint(1))/...
                     (blockpoint(2)-gripPos(2)))- youbotEuler(3);
                 if (blockpoint(2)-gripPos(2)) > 0
                     NewAngl = NewAngl + pi;
                 end
            
       
                %put the gripper in vertical position
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(2), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);

                res = vrep.simxSetJointTargetPosition(id, h.armJoints(3), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);

                res = vrep.simxSetJointTargetPosition(id, h.armJoints(4), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);

                res = vrep.simxSetJointTargetPosition(id, h.armJoints(5), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                pause(1);

                % orient the gripper to the block
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(1), NewAngl, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                pause(1)
                fsm = 'move2gras';

            
            elseif strcmp(fsm, 'move2gras')
                
%                                  % Variables of the youbot
%                 l1 = 0.147+0.0952; % initial height
%                 l2 = 0.155; % length of the first arm 
%                 l3 = 0.135; % length of the second arm 
%                 l4 = 0.1606 ;%length of the third arm
% 
%                 gripPos(3) = l1;
% 
%                 h_block = blockpoint(3) - gripPos(3);
%                 r_block = sqrt((blockpoint(1) - gripPos(1))^2 + (blockpoint(2) - gripPos(2))^2);
% 
%                 syms theta2 theta3
%                 eq1 = l2*sin(theta2) + l3*sin(theta2 + theta3) + l4 == r_block;
%                 eq2 = l2*cos(theta2) + l3*cos(theta2 + theta3) == h_block;
% 
%                 S = vpasolve([eq1 eq2], [theta2 theta3], [-1.5707963705063, 1.308996796608; 0, 2.2863812446594]);
%                 intertheta2 = S.theta2;
%                 intertheta3 = S.theta3;
% 
%                 theta4  = pi/2 - intertheta2(1) - intertheta3(1);
% 
%                 res = vrep.simxSetJointTargetPosition(id, h.armJoints(3), intertheta3(1), vrep.simx_opmode_oneshot);
%                 vrchk(vrep, res, true);
% 
%                 res = vrep.simxSetJointTargetPosition(id, h.armJoints(4), theta4, vrep.simx_opmode_oneshot);
%                 vrchk(vrep, res, true);
%                 pause(1);
% 
%                 res = vrep.simxSetJointTargetPosition(id, h.armJoints(2), intertheta2(1), vrep.simx_opmode_oneshot);
%                 vrchk(vrep, res, true);
%                 pause(2);
% 
%                 fsm ='grasp';
    
                 % Variables of the youbot
                l1 = 0.147+0.0952; % initial height
                l2 = 0.155; % length of the first arm 
                l3 = 0.135; % length of the second arm 
                l4 = 0.141;%0.142;%length of the third arm

                gripPos(3) = l1;

                h_block = blockpoint(3) - gripPos(3);
                r_block = sqrt((blockpoint(1) - gripPos(1))^2 + (blockpoint(2) - gripPos(2))^2);

                syms theta2 theta3
                eq1 = l2*sin(theta2) + l3*sin(theta2 + theta3) + l4 == r_block - 0.04;
                eq2 = l2*cos(theta2) + l3*cos(theta2 + theta3) == h_block;

                S = vpasolve([eq1 eq2], [theta2 theta3], [-1.5707963705063, 1.308996796608; 0, 2.2863812446594]);
                intertheta2 = S.theta2(1);
                intertheta3 = S.theta3(1);

                theta4  = pi/2 - intertheta2 - intertheta3;

                res = vrep.simxSetJointTargetPosition(id, h.armJoints(3), intertheta3, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);

                res = vrep.simxSetJointTargetPosition(id, h.armJoints(4), theta4, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                pause(2);

                res = vrep.simxSetJointTargetPosition(id, h.armJoints(2), intertheta2, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                pause(2);
                
                
                syms theta2 theta3
                eq1 = l2*sin(theta2) + l3*sin(theta2 + theta3) + l4 == r_block;
                eq2 = l2*cos(theta2) + l3*cos(theta2 + theta3) == h_block;

                S = vpasolve([eq1 eq2], [theta2 theta3], [-1.5707963705063, 1.308996796608; 0, 2.2863812446594]);
                intertheta2 = S.theta2(1);
                intertheta3 = S.theta3(1);

                theta4  = pi/2 - intertheta2 - intertheta3;
                
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(3), intertheta3, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);

                res = vrep.simxSetJointTargetPosition(id, h.armJoints(4), theta4, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                pause(0.3);
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(2), intertheta2, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                pause(2);

                fsm ='grasp';
            
            elseif strcmp(fsm, 'grasp')
                % Close the gripper. Please pay attention that it is not possible to adjust the force to apply:  
                % the object will sometimes slip from the gripper!
                res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);

                % Make MATLAB wait for the gripper to be closed. This value was determined by experiments. 
                pause(2);  

                % Go back to rest position.
                % Set each joint to their original angle, as given by startingJoints.
                for i = 1:5
                    res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
                    vrchk(vrep, res, true);
                end
                pause(2);

                fsm = 'moveTable4'; 

            elseif strcmp(fsm, 'moveTable4')

                blockClosePoint = [youbotPos(1) youbotPos(2)];
                initialDistSlide = sqrt((blockFarPoint(1) -  blockClosePoint(1))^2 ...
                        +(blockFarPoint(2) -  blockClosePoint(2))^2);
                interPosSlide = blockClosePoint;
                prevPosSlide = blockClosePoint;

                fsm = 'slide';
                interfsm  = 'findbasket';


            elseif strcmp(fsm, 'findbasket')
                %%  
                %method to find correct table
                correctpicture = false;
                basket_number = 0;
                for i=1:length(inst)
                    if strcmp (formBlock, inst(i).shape)
                        if strcmp (colorName,inst(i).colorname)
                            for j=1:length(pictureBasket)
                                if strcmp (char(pictureBasket(j)),inst(i).picture)
                                    correctpicture = true
                                    basket_number = j
                                    pictureBasket(j)
                                    NewGoal = basketGoal(j,:)
                                    break;
                                end
                            end
                        end
                    end
                end
                
                if ~correctpicture
                    disp('Failed recognition table -> go to table 1')
                    basket_number = 1
                    pictureBasket(1)
                    NewGoal = basketGoal(1,:)
                end

                step = 'travel';
                fsm = 'newdestination';
                interfsm = 'basket_rotate';
                  
            elseif strcmp(fsm, 'basket_rotate')
                NewAngl = atan(((resol*youbotPos(1)-minMap(1)+1)-basketCenter(basket_number,1))/...
                    (basketCenter(basket_number,2)-(resol*youbotPos(2)-minMap(2)+1))) - pi/2 ;
                if (basketCenter(basket_number,2)-(resol*youbotPos(2)-minMap(2)+1)) > 0
                    NewAngl = NewAngl + pi;
                end

                step = 'travel';
                fsm = 'rotate';
                interfsm = 'basket_slide';
                interval = 'only_rotate';

             elseif strcmp(fsm, 'basket_slide')
                   
                % get rgb camera position
                [res, rgbdPos] = vrep.simxGetObjectPosition(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);

                NewAngl = atan(((resol*rgbdPos(1)-minMap(1)+1)- basketCenter(basket_number,1))/...
                    (basketCenter(basket_number,2)-(resol*rgbdPos(2)-minMap(2)+1))) - pi/2 ;
                if (basketCenter(basket_number,2)-(resol*rgbdPos(2)-minMap(2)+1)) > 0
                    NewAngl = NewAngl + pi;
                end
                vrep.simxSetObjectOrientation(id, h.rgbdCasing, -1,[0 0 NewAngl], vrep.simx_opmode_oneshot);

                % get rgb camera angle
                [res, rgbdEuler] = vrep.simxGetObjectOrientation(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);    
                % medium view to identify basket
                res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/5, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);

                % take a 3D image
                res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
                % and store this image in pts
                pts_basket = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
                % Each column of pts has [x;y;z;distancetosensor]. However, plot3 does not have the same frame of reference as 
                % the output data. To get a correct plot, you should invert the y and z dimensions. 

                %we only keep points within 1.5 meter, to focus on the basket. 
                pts_basket = pts_basket(1:3, pts_basket(4, :) < 1.5);

                % transformation in the right pose
                pts_basket = [pts_basket(3,:);pts_basket(1,:);pts_basket(2,:)];
                trf = transl(rgbdPos) * trotx(rgbdEuler(1)) * troty(rgbdEuler(2)) * trotz(rgbdEuler(3));
                pts_basket = homtrans(trf, pts_basket);

                % the height of the basket is 185 mm
                % we conserv point with a height between 50 and 100 mm by security
                % to identify the table and not see the robot with the captor
                pts_basket = pts_basket(1:3,pts_basket(3,:) < 0.10);
                pts_basket = pts_basket(1:2,pts_basket(3,:) > 0.05);

                indexBasket = find(bsxfun(@hypot, pts_basket(:,1)- youbotPos(1), pts_basket(:,2)-youbotPos(2))==...
                    min(bsxfun(@hypot, pts_basket(:,1)-youbotPos(1), pts_basket(:,2)-youbotPos(2))),1,'last')

                basketDist = sqrt((pts_basket(1, indexBasket)-youbotPos(1))^2 + ...
                    (pts_basket(2, indexBasket)-youbotPos(2))^2)

                blockFarPoint = [youbotPos(1) youbotPos(2)]; 

%                 AnglDestBlock = atan((pts_basket(2, indexBasket) - blockFarPoint(2))/...
%                     (pts_basket(1, indexBasket) - blockFarPoint(1))) ;
%                 if (pts_basket(1, indexBasket) - blockFarPoint(1)) > 0
%                     AnglDestBlock = AnglDestBlock + pi;
%                 end
% 
%                 blockClosePoint = [(pts_basket(1, indexBasket) + 0.35 * cos(AnglDestBlock))...
%                     (pts_basket(2, indexBasket) + 0.35 * sin(AnglDestBlock))];
%                 
                
        
               AnglDestBlock = atan((basketCenter(basket_number,2) - (resol*youbotPos(2)-minMap(2)+1))/...
                    (basketCenter(basket_number,1) - (resol*youbotPos(1)-minMap(1)+1))) ;
                if (basketCenter(basket_number,1) - (resol*youbotPos(1)-minMap(1)+1)) > 0
                    AnglDestBlock = AnglDestBlock + pi;
                end

                blockClosePoint = [(youbotPos(1) - (basketDist - 0.4) * cos(AnglDestBlock))...
                    (youbotPos(2) - (basketDist - 0.4) * sin(AnglDestBlock))];

  
                initialDistSlide = sqrt((blockFarPoint(1) -  blockClosePoint(1))^2 ...
                        +(blockFarPoint(2) -  blockClosePoint(2))^2);
                interPosSlide = blockFarPoint;
                prevPosSlide = blockFarPoint;
                
                figure;
                 plot(pts_basket(1,:), pts_basket(2,:), '.',rgbdPos(1), rgbdPos(2),'+',...
                     youbotPos(1),  youbotPos(2), '-', pts_basket(1, indexBasket),...
                     pts_basket(1, indexBasket), '^' );

                fsm = 'slide';
                interfsm  = 'basket_movearm';

            elseif strcmp(fsm, 'basket_movearm')

                % get rgb position
                [res, rgbdPos] = vrep.simxGetObjectPosition(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
                    
                gripPos = [0 0.166 0] ;
                trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
                gripPos = homtrans(trf, transpose(gripPos)); 
             
                 % rotation to the block on the table
                 (resol*gripPos(1)-minMap(1)+1)
                 basketCenter(basket_number,1)
                   (resol*gripPos(2)-minMap(2)+1)
                 basketCenter(basket_number,2)
                 NewAngl = atan(((resol*youbotPos(1)-minMap(1)+1) - basketCenter(basket_number,1))/...
                     (basketCenter(basket_number,2) - (resol*youbotPos(2)-minMap(2)+1)))- youbotEuler(3)-pi/8;
                 if (basketCenter(basket_number,2) - (resol*youbotPos(2)-minMap(2)+1)) > 0
                     NewAngl = NewAngl + pi;
                 end
                 
                 figure;
                 plot(pts_basket(1,:), pts_basket(2,:), '.',rgbdPos(1), rgbdPos(2), '^', gripPos(1), gripPos(2), '+',...
                     youbotPos(1),  youbotPos(2), '-', pts_basket(1, indexBasket),...
                     pts_basket(1, indexBasket), '^' );
                 
%                  figure;
%                 plot(pts_basket(1,:), pts_basket(2,:), '*',pts_basket(1, indexBasket), ...
%                     pts_basket(2, indexBasket), '.', youbotPos(1), youbotPos(2), '+')

                %put the gripper in vertical position
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(2), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);

                res = vrep.simxSetJointTargetPosition(id, h.armJoints(3), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);

                res = vrep.simxSetJointTargetPosition(id, h.armJoints(4), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);

                res = vrep.simxSetJointTargetPosition(id, h.armJoints(5), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                pause(1.5);

                % orient the gripper to the block
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(1), NewAngl, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                pause(1)
                     
            
                %put the gripper in vertical position
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(2),  70 * pi / 180, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);

                res = vrep.simxSetJointTargetPosition(id, h.armJoints(3),  20 * pi / 180, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                pause(2)

                fsm  = 'basket_movearm2';

            
            elseif strcmp(fsm, 'basket_movearm2')
                
                res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
                pause(2);  

                % Go back to rest position.
                % Set each joint to their original angle, as given by startingJoints.
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(2), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);

                res = vrep.simxSetJointTargetPosition(id, h.armJoints(3), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                pause(0.5)
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(1), startingJoints(1), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                pause(1)
                
                    % Go back to rest position.
                % Set each joint to their original angle, as given by startingJoints.
                for i = 2:5
                    res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
                    vrchk(vrep, res, true);
                end
                pause(2);

                
                fsm = 'basket_movearm3'; 
                
            elseif strcmp(fsm, 'basket_movearm3')

                blockClosePoint = [youbotPos(1) youbotPos(2)];
                initialDistSlide = sqrt((blockFarPoint(1) -  blockClosePoint(1))^2 ...
                        +(blockFarPoint(2) -  blockClosePoint(2))^2);
                interPosSlide = blockClosePoint;
                prevPosSlide = blockClosePoint;

                fsm = 'slide';
                interfsm  = 'moveTable';
            end

        % Exit the function.
        elseif strcmp(step, 'finished')
            pause(3);
            break;
        else
            error('Unknown state %s.', fsm);
        end
        
        % Update wheel velocities using the global values (whatever the state is). 
        h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);
        drawnow;
        % Make sure that we do not go faster than the physics simulation (each iteration must take roughly 50 ms). 
        elapsed = toc;
        timeleft = timestep - elapsed;
        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end   
   
end % Main function.