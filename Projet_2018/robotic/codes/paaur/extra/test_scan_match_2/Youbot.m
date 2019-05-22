% ------ TODOS ------- %
% 1. Clean code
% 2. Termination robot when map finished
% 3. Save with name in 'maps' folder
% 4. Testing recovery
% 5. Testing on several configurated maps
% 6. Commenting all files with code
% 7. GUI legend added to the plots
% 8. Readme
% 9. Rotate faster


classdef Youbot

    properties (Constant)
        ROBOT_RADIUS = 1.05; % Radius of robot 
        OBSERVE_ANGLE = pi; % Observation angle
        GOAL_RADIUS = .6; % Defines if we are close enough to goal % 0.65 % 0.75
        MAX_RANGE = 5; % Range of laser
        T_SIM = .05; % Time step of simulator (don't change)
        T_UI = 25; % Time step for update of UI
        K = 2.5; % velocity profile parameter 
        R = .85; % velocity profile parameter       
        FREE_THRES = .45; % Probability threshold for free cells in occupancy grid
        OCC_THRES = .55; % Probability threshold for occupied cells in occupancy grid
        NB_NODES = 100; % Number of random nodes in probalistic roadmap
        CONNECTION_DISTANCE = 3.2; % Maximum connection distance between nodes in probalistic roadmap
        INFLATE_INIT = .35; % Inflate factor used for preparing maps for frontier-based approach % 0.35
        INFLATE_FRONTIER = .15; % Inflate factor used for getting frontiers for frontier-based approach % 0.2
        INFLATE_PRM = .4; % Inflate factor used for path finding % 0.5
        ALPHA = .25; % Importance of distance brut
        BETA = .75; % Importance of correction
        N = 16; % Dimension of map (NxN) - for display: change accordingly in appdesigner dashboard

        % Minimum and maximum angles for all joints. Only useful to implement custom IK. 
        armJointRanges = [-2.9496064186096, 2.9496064186096;
                          -1.5707963705063, 1.308996796608;
                          -2.2863812446594, 2.2863812446594;
                          -1.7802357673645, 1.7802357673645;
                          -1.5707963705063, 1.5707963705063 ];

        % Definition of the starting pose of the arm.
        startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
    end
    
    properties (Access = private)
        app % reference to User Interface
        cleanupObj % Object that makes sure connenction with simulation is properly closed
        vrep % API to simulator
        id % id of robot object
        h % handles of robot
        pickupJoints % The preset pickup pose
        forwBackVel % Velocity of forward-backward movement of robot
        leftRightVel % Velocity of left-right movement of robot
        rotVel % Velocity of rotation movement of robot
        x % Pose of robot (X,Y,theta)
        prevx % Previous pose of robot
        counter % Counter of steps
        counterHelper % Another counter used in assosciation with 'counter' for recovery 
        fsm % State of the robot
        prevfsm % Previous state of the robot
        m % Probabilistic occupancy map of environment
        angle % Angle of rotation that needs to be done by robot
        goal % Goal position (X,Y)
        path % Path leading from start to goal 
        iter % Iterator over points of path
        flagNewGoal % Flag 1 if new goal needs to be computed, 0 otherwise
        frnts % Frontier points to display 
        nbRecovery % Counter of number of times robot was in recovery mode
        controller
        
        rngs
        angls
        tmp_rngs
        tmp_angls
        tmp_pose
        tmp_odometry 
    end

    methods (Access = public)
        %% Constructor of Explorer
        function obj = Youbot(app)
            obj.app = app; 
            
            % Use the following line if you had to recompile remoteApi
            %vrep = remApi('remoteApi', 'extApi.h');
            obj.vrep = remApi('remoteApi');
            obj.vrep.simxFinish(-1);
            obj.id = obj.vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

            % If you get an error like: 
            %   Remote API function call returned with error code: 64. Explanation: simxStart was not yet called.
            % Make sure your code is within a function! You cannot call V-REP from a script. 
            if obj.id < 0
                disp('Failed connecting to remote API server. Exiting.');
                obj.vrep.delete(); % USE
                return;
            end

            % Make sure we close the connection whenever the script is interrupted.
            obj.cleanupObj = onCleanup(@() cleanup_vrep(obj.vrep, obj.id));

            % This will only work in "continuous remote API server service". 
            % See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
            obj.vrep.simxStartSimulation(obj.id, obj.vrep.simx_opmode_oneshot_wait);

            % Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose.
            obj.h = youbot_init(obj.vrep, obj.id);
            obj.h = youbot_hokuyo_init(obj.vrep, obj.h);

            % Let a few cycles pass to make sure there's a value waiting for us next time
            % we try to get a joint angle or the robot pose with the simx_opmode_buffer
            % option.
            pause(.2);

            obj.pickupJoints = [90 * pi / 180, 19.6 * pi / 180, 113 * pi / 180, - 41 * pi / 180, 0];
            obj.forwBackVel = 0;
            obj.leftRightVel = 0;
            obj.rotVel = 0;

            % Set the arm to its starting configuration. 
            res = obj.vrep.simxPauseCommunication(obj.id, true); % Send order to the simulator through vrep object. 
            vrchk(obj.vrep, res); % Check the return value and exit in case of error. 
            for i = 1:5
                res = obj.vrep.simxSetJointTargetPosition(obj.id, obj.h.armJoints(i), obj.startingJoints(i), obj.vrep.simx_opmode_oneshot);
                vrchk(obj.vrep, res, true);
            end
            res = obj.vrep.simxPauseCommunication(obj.id, false); 
            vrchk(obj.vrep, res);
            
            % Initialise general variables
            obj = obj.pose(); % Initialise pose
            obj = obj.pose(); % Make sure the previous pose is equal to the initial pose (see method pose)
            obj.goal = obj.x(1:2); % Set goal to initial pose
            obj.iter = 1; 
            obj.counter = 0;
            obj.counterHelper = 0;
            obj.fsm = 'control'; % Start state
            obj.prevfsm = 'none'; % No previous state
            obj.flagNewGoal = 1; % Initially a new goal needs to be computed
            obj.nbRecovery = 0; 

            % Initialise map
            obj.m = robotics.OccupancyGrid(obj.N,obj.N,5); % 10 % 5!!!
            obj.m.GridLocationInWorld = [-obj.N/2 -obj.N/2];
            obj.m.FreeThreshold = obj.FREE_THRES;
            obj.m.OccupiedThreshold = obj.OCC_THRES;
            setOccupancy(obj.m, obj.x(1:2), 0); % initial position is free on map
            
            % Update user interface values
            obj.app.poseX.Value = obj.x(1);
            obj.app.poseY.Value = obj.x(2);
            obj.app.goalX.Value = obj.goal(1);
            obj.app.goalY.Value = obj.goal(2);
            
            obj.controller = robotics.PurePursuit;
            
            obj.tmp_rngs = [];
            obj.tmp_angls = [];
            obj.tmp_pose = [];
            obj.tmp_odometry = [];
            
            % Make sure everything is settled before we start. 
            pause(2);
        end
        
        %% Save map to directory 'maps'
        function save(obj)
            mat = obj.m.occupancyMatrix();
            save(strcat('maps/', char(java.util.UUID.randomUUID)), 'mat');
            
            t = obj.tmp_pose;
            t;
            rngs_ = obj.tmp_rngs;
            rngs_;
            angls_ = obj.tmp_rngs;
            pose_ = obj.tmp_rngs;
            odometry_ = obj.tmp_rngs;
            save('data/result', 'rngs_', 'angls_', 'pose_', 'odometry_');
        end
        
        %% Exploration of unknown environment
        function obj = exploration(obj)

            while obj.app.onoff.Value == 1 % Flag in UI controlled by start/stop button
                tic  
                if obj.vrep.simxGetConnectionId(obj.id) == -1
                  error('Lost connection to remote API.');
                end

                obj = obj.pose(); % Update pose

                % State machine
                switch obj.fsm
                    case 'control'
                        obj = obj.state_control();
                    case 'goal'
                        obj = obj.state_goal();
                    case 'rotate'
                        obj = obj.state_rotate();
                    case 'move'
                        obj = obj.state_move();
                    case 'recovery'
                        obj = obj.state_recovery();
                    case 'finish'
                        obj = obj.state_finish();
                    otherwise
                        pause(1);
                        break;
                end

                % Update wheel velocities using the global values (whatever the state is). 
                obj.h = youbot_drive(obj.vrep, obj.h, obj.forwBackVel, obj.leftRightVel, obj.rotVel);
                
                % Update UI - each 25 cycles * 
                if (mod(obj.counter,obj.T_UI) == 0)
                    obj.app.poseX.Value = obj.x(1); % Update pose in UI
                    obj.app.poseY.Value = obj.x(2); % Update pose in UI
                    
                    % Plot used for visualizing mapping done by robot in
                    % real-time
                    an = obj.app.UINavigation;
                    domain = occupancyMatrix(obj.m,'ternary');
                    a = world2grid(obj.m, obj.x(1:2));
                    c = world2grid(obj.m, obj.goal);
%                     if ~isempty(obj.path)
%                        d = world2grid(obj.m, obj.path);
%                     end
                    domain(a(1),a(2)) = 0.1;
                    domain(c(1),c(2)) = 0.3;
                    
%                     if ~isempty(obj.path)
%                         domain(d(:,1),d(:,2)) = 0.4;
%                     end
                    domain = flipud(domain);
                    [occX, occY] = find(domain ~= 0);
                    [aY, aX] = find(domain == 0.1);
                    [cY, cX] = find(domain == 0.3);
                    
%                     if ~isempty(obj.path)
%                         [dY, dX] = find(domain == 0.4);
%                     end
                    if isempty(obj.frnts)
                        plot(an, occY, occX, 'k+', aX, aY, 'r*', cX, cY, 'b*');
                    elseif ~isempty(obj.frnts) && ~isempty(obj.path)
                        d = world2grid(obj.m, obj.path);
                        plot(an, occY, occX, 'k+', obj.frnts(:,2), size(domain,1)-obj.frnts(:,1),'co', aX, aY, 'r*', cX, cY, 'b*', d(2:4:end,2), size(domain,1)-d(2:4:end,1), 'go'); 
                    else
                        plot(an, occY, occX, 'k+', obj.frnts(:,2), size(domain,1)-obj.frnts(:,1),'co', aX, aY, 'r*', cX, cY, 'b*'); 
                    end
                    drawnow;
                    
                    % Plot used for visualizing mapping done by robot in
                    % real-time
                    am = obj.app.UIMap;
                    domain = occupancyMatrix(obj.m);
                    domain = flipud(domain);
                    [obsX, obsY] = find(domain >= 0.7);
                    plot(am, obsY, obsX, 'k+');
                    drawnow;
                end

                % Make sure that we do not go faster that the simulator (each iteration must take 50 ms). 
                elapsed = toc;
                timeleft = obj.T_SIM - elapsed; %% TO change
                if timeleft > 0
                    pause(min(timeleft, .01));
                end
                obj.counter = obj.counter+1;
            end % End while
        end % End function exploration 
        
        function obj = stop(obj)
            cleanup_vrep(obj.vrep, obj.id);
        end
 
    end % End public methods
       
    methods (Access = private)
        %% Control state
        function obj = state_control(obj)
            if strcmp(obj.fsm, 'finish')
                return;
            end
            
            distanceToGoal = norm(obj.x(1:2) - obj.goal);

            if ~obj.flagNewGoal && distanceToGoal <= obj.GOAL_RADIUS % If we are close enough to goal
                obj.iter = 1;
                obj.fsm = 'control';
                obj.flagNewGoal = 1;                  
            elseif obj.flagNewGoal && strcmp(obj.prevfsm, 'rotate') % Compute new goal
                obj.fsm = 'goal';
            elseif ~obj.flagNewGoal  % Move to next point
                obj.fsm = 'rotate';
            else % Observe rotate
                obj.fsm = 'rotate';
                mat = occupancyMatrix(obj.m,'ternary'); 
                f = ConvFilter(mat, 50); % Instantiate filter
                a = f.get_cost(world2grid(obj.m,obj.x(1:2))); % Get the costs for actual position of robot
                if a < 0.35 % Region around robot is unknown -> observe
                    obj.angle = obj.x(3) + obj.OBSERVE_ANGLE;
                else
                    obj.angle = obj.x(3) + 0;
                end
            end
            
            obj.prevfsm = 'control';
            
            % Update UI
            obj.UI_update_state();
        end
        
        %% Goal state 
        function obj = state_goal(obj)
            obj = obj.compute_goal(obj.x(1:2)); % Compute goal
            if isempty(obj.goal)
                return;
            end
            
            obj.path = obj.path_dstar(obj.x(1:2), obj.goal); % Compute path corresponding to goal
            if ~isempty(obj.goal) && ~isempty(obj.path)
                obj.flagNewGoal = 0; % New goal is computed 
                obj.fsm = 'control';
                obj.prevfsm = 'goal';
                obj.UI_update_state(); % Update state
            end
            
            obj.controller.Waypoints = obj.path(2:4:end,:); %:4:
            obj.controller.DesiredLinearVelocity = 1.85; % 1.5
            obj.controller.MaxAngularVelocity = 0.4; % 0.4
            obj.controller.LookaheadDistance = 1; % 1 
            
            e = atan2(obj.path(2,2)-obj.x(2),obj.x(1)-obj.path(2,1)); % angle between sensor and target
            obj.angle = -(e+pi/2); 
        end
        
        %% Rotation state
        function obj = state_rotate(obj)
            obj.rotVel = angdiff(obj.angle, obj.x(3)); % Rotation velocity is proportional to difference between target angle and actual angle
            
            % IF-condition 1 : rotation velocity is slow enough
            % IF-condition 2 : rotation displacement is small enough
            if (abs(obj.rotVel) < .035 / 180 * pi) && (abs(angdiff(obj.prevx(3), obj.x(3))) < .035 / 180 * pi) % (abs(angdiff(obj.prevx(3), obj.x(3))) < .01 / 180 * pi)
                obj.rotVel = 0; % Stop rotation
                if obj.flagNewGoal
                    obj.fsm = 'control';
                else 
                    obj.fsm = 'move';
                end
                obj.prevfsm = 'rotate';
                obj.UI_update_state();
            end
            obj = obj.mapping(); % Update mapping with laser scans
        end
       
        %% Move state
        function obj = state_move(obj)
            d = norm(obj.x(1:2) - obj.goal(1:2)); % distance between actual position and goal 
            
            [ranges, angles] = sensor(obj.vrep, obj.h, obj.vrep.simx_opmode_buffer, 2, obj.x); % Get laser scans of sensor 2
            rangesFront = ranges(angles < pi/180 * 7.5 & angles > -pi/180 * 7.5); % Take only the 15 degrees in front
            obsFront = rangesFront(rangesFront < 0.2);

            % Deceleration if near to goal
            if d > obj.GOAL_RADIUS && d <= obj.GOAL_RADIUS+0.11
                obj.controller.DesiredLinearVelocity = obj.controller.DesiredLinearVelocity*0.8;
            end
            
            if d > obj.GOAL_RADIUS 
                % Compute the controller outputs, i.e., the inputs to the robot
                [v, omega] = obj.controller([obj.x(1:2) (obj.x(3)-pi/2)]);

                % Simulate the robot using the controller outputs.
                obj.forwBackVel = -v;
                obj.rotVel = omega;
            else
                obj.forwBackVel = 0;
                obj.rotVel = 0;
                obj.counterHelper = obj.counterHelper + 1;
                if mod(obj.counterHelper,15) == 0
                    obj.fsm = 'control';
                    obj.counterHelper = 0;
                    obj.prevfsm = 'move';
                    obj.UI_update_state();
                end
            end
            
            % IF-condition: obstacle in front of robot
            if ~isempty(obsFront) % See an obstacl
                obj.forwBackVel = 0;
                obj.rotVel = 0;
                obj.counterHelper = obj.counterHelper + 1;
                if mod(obj.counterHelper,15) == 0
                    obj.fsm = 'recovery';
                    obj.counterHelper = obj.counter;
                    obj.prevfsm = 'move';
                    obj.UI_update_state();
                end
            end
            
            % Mapping - each 5 cycles * 
            if (mod(obj.counter,5) == 0)
                obj = obj.mapping();
                
                
                obj.tmp_rngs = [obj.tmp_rngs; obj.rngs];
                obj.tmp_angls = [obj.tmp_angls; obj.angls];
                obj.tmp_pose = [obj.tmp_pose; obj.x];
                obj.tmp_odometry = [obj.tmp_odometry; obj.forwBackVel obj.rotVel];
            end
            
            if (mod(obj.counter,300) == 0)
                rngs_ = obj.tmp_rngs;
                angls_ = obj.tmp_angls;
                pose_ = obj.tmp_pose;
                odometry_ = obj.tmp_odometry;
                save('data/result', 'rngs_', 'angls_', 'pose_', 'odometry_');
            end
        end
        
        %% Recovery state
        function obj = state_recovery(obj)
            obj.forwBackVel = obj.velocity(0.4); % Move in opposite direction
            
            % After 50 time steps stop recovery
            if (obj.counter == obj.counterHelper + 5)
                obj.forwBackVel = 0;
                obj.flagNewGoal = 1; % New goal needs to be computed
                obj.fsm = 'control';
                obj.counterHelper = 0;
                obj.prevfsm = 'recovery';
                obj.nbRecovery = obj.nbRecovery + 1;
                obj.UI_update_state();
            end
        end
        
        %% Finish state 
        function obj = state_finish(obj)
            obj.forwBackVel = 0;
            obj.leftRightVel = 0;
            obj.rotVel = 0;
            obj = obj.stop();
        end
        
        %% Compute pose of robot
        function obj = pose(obj)
            obj.prevx = obj.x;
            [res, youbotPos] = obj.vrep.simxGetObjectPosition(obj.id, obj.h.ref, -1, obj.vrep.simx_opmode_buffer);
            vrchk(obj.vrep, res, true);
            [res, youbotEuler] = obj.vrep.simxGetObjectOrientation(obj.id, obj.h.ref, -1, obj.vrep.simx_opmode_buffer);
            vrchk(obj.vrep, res, true);
            obj.x = [youbotPos(1), youbotPos(2), youbotEuler(3)];
        end
        
        %% Update occupancy map with laser scans data
        function obj = mapping(obj)
            obj.rngs = [];
            obj.angls = [];
            for id_sensor = 1:2 % Number of laser sensors
                [ranges, angles] = sensor(obj.vrep, obj.h, obj.vrep.simx_opmode_buffer, id_sensor, obj.x); % Get observations from sensor
                obj.rngs = [obj.rngs ranges];
                obj.angls = [obj.angls angles];
                insertRay(obj.m, obj.x, ranges, angles, obj.MAX_RANGE); % Actualize maps according to observations
            end
        end
        
        %% Path planning using probabilistic roadmap algorithm
        function path = path_dstar(obj, start, dest)
            inflateFactor = obj.INFLATE_PRM;
            while true
                map = copy(obj.m);
                inflate(map, obj.ROBOT_RADIUS*inflateFactor); % Inflate map
                if (getOccupancy(map, start) < map.FreeThreshold) & (getOccupancy(map, dest) < map.FreeThreshold) % Valid inflation
                    break;
                end
                inflateFactor = inflateFactor*0.9; % Diminish inflateFactor
            end
            
            start = world2grid(map, start);
            start = [start(2) start(1)];
            dest = world2grid(map, dest);
            dest = [dest(2) dest(1)];
            mat = occupancyMatrix(map, 'ternary');
            mat(mat == -1) = 1;

            ds = Dstar(mat);   % create navigation object
            ds.plan(dest);       % create plan for specified goal
            path = ds.path(start);      % animate path from this start location

            p = [path(:,2) path(:,1)];
            path = grid2world(map, p);
        end

        %% Velocity profile of robot
        function v = velocity(obj, dist)
            k = obj.K * 0.9^obj.nbRecovery; % Dimish velocity if high number of recoveries
            v = max(obj.K*exp(-obj.R/dist), .007);
        end
        
        function dist = dist_prm(obj, start, dest)
            prm = robotics.PRM;
            prm.Map = obj.m;
            p = [];
            while isempty(p)
                p = findpath(prm, double(start), dest);
                prm.NumNodes = prm.NumNodes+10;
            end
            dist = 0;
            for i = 2:size(p,1)
                dist = dist + norm(p(i,:)-p(i-1,:));
            end
        end
        
        
        %% Compute goal using adapted frontier-based approach  
        function obj = compute_goal(obj, start)
            % Inflate map
            inflateFactor = obj.INFLATE_INIT;
            while true
                map = copy(obj.m);
                inflate(map, obj.ROBOT_RADIUS*inflateFactor);
                if (getOccupancy(map, start) < map.FreeThreshold) % Valid inflation
                    break;
                end
                inflateFactor = inflateFactor*0.9; % Diminish inflateFactor
            end
            
            % Frontier detection
            
            % Create m1 
            % 1 -> 0
            % 0 -> 0
            % -1 -> 1
            mat = occupancyMatrix(map,'ternary');
            mat(mat == 1) = 0;
            mat(mat == -1) = 1;
            m1 = robotics.OccupancyGrid(mat);

            % Create m2
            % 1 -> 1
            % 0 -> 0
            % -1 -> 1
            mat = occupancyMatrix(map,'ternary');
            mat(mat == -1) = 1;
            m2 = robotics.OccupancyGrid(mat);
            
            % Inflation of m1
            inflate(m1, obj.INFLATE_FRONTIER);
            
%             figure(4)
%             subplot(4,1,1)
%             show(map)
%             subplot(4,1,2)
%             show(m1)
%             subplot(4,1,3)
%             show(m2)
%             subplot(4,1,4)
%             show(m3)
            
            % Intersection of m1 & m2 (frontiers) but with m3 to 0 (without obstacles)
            mat1 = occupancyMatrix(m1,'ternary');
            mat2 = occupancyMatrix(m2,'ternary');
            [r,c] = find(mat1 == 1 & mat2 == 0); % frontiers of unknown points 
            pts = grid2world(map, [r,c]); % coordinates of frontier pts
            
            % Frontier selection (pass I)
            
            % Apply first filter to check if we have ever
            mat = occupancyMatrix(obj.m,'ternary'); % Not inflated!
            f = ConvFilter(mat, 25); % Instantiate filter used for determining where are the most unknown regions
            cst = f.get_cost(world2grid(obj.m,pts)); % Get the costs for the frontiers
            pts = pts(cst<0.6,:); % Selection of frontiers % 0.65
            if isempty(pts) % No points to explore anymore
                obj.fsm = 'finish';
                obj.UI_update_state();
                return;
            end
            
            % Frontier selection (pass II)
            pt = floor(pts/35);
            regions = unique(pt, 'rows');
            mindist = Inf;
            goali = [0 0];
            for j = 1:size(regions,1)
                n_pts_in_region = sum(ismember(pt,regions(j,:),'rows'));
                idx = round(n_pts_in_region*rand+0.5);
                idx = find(pt(:,1) == regions(j,1) & pt(:,2) == regions(j,2),idx,'first');
                selected_point = pts(idx(end),:);
                dist = obj.dist_prm(start, selected_point);
                if mindist > dist 
                    mindist = dist;
                    goali = selected_point;
                end
            end
            obj.goal = goali;
            
            % Update UI
            obj.app.goalX.Value = obj.goal(1);
            obj.app.goalY.Value = obj.goal(2); 
            obj.frnts = world2grid(obj.m,pts);
        end
        
        %% Update state in User Interface
        function obj = UI_update_state(obj)
            obj.app.state.Text = obj.fsm; 
        end
        
    end % End private methods
end