function map()
	% Initiate the connection to the simulator.
	% addpath('C:\Users\MonOrdi\Documents\trs\youbot');
	addpath('C:\trs\youbot');
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
	% The tip corresponds to the point between the two tongs of the gripper (for more details, see later or in the
	% file focused/youbot_arm.m).
	h = youbot_init(vrep, id);
	h = youbot_hokuyo_init(vrep, h);

	% Let a few cycles pass to make sure there's a value waiting for us next time we try to get a joint angle or
	% the robot pose with the simx_opmode_buffer option.
	pause(.2);

	%% Youbot constants
	% The time step the simulator is using (your code should run close to it).
	timestep = .05;

    % Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
	startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
       
	%% Preset values for the demo.
	disp('Starting robot');

	% Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels.
	% They are adapted at each iteration by the code.
	forwBackVel = 0;        % Move straight ahead.
	rightVel = 0;           % Go sideways.
	rotateRightVel = 0;     % Rotate.
	prevOrientation = 0;    % Previous angle to goal (easy way to have a condition on the robot's angular speed).
	prevPosition = 0;       % Previous distance to goal (easy way to have a condition on the robot's speed).
	INITIAL_MAPPING = false;
	MAX_SPEED = 40; % determine a maximal speed for the robot
	MAX_DISTANCE = 50;  % acceleration over 50 meters
    trajectory = [];
    FIRST_TRAJ = true;
    
	res = vrep.simxPauseCommunication(id, true);
	vrchk(vrep, res);
    
	% Set the target position of a joint.
	for i = 1:5
		res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
		vrchk(vrep, res, true);
	end
	res = vrep.simxPauseCommunication(id, false); 
	vrchk(vrep, res);
	
	% Initialise the map.
	UNEXPLORED_CELL = 1;
	FREE_CELL = 0;
	BLOCKED_CELL = 2;
	% plan is a vector containing the map of the explorated house. The plan as
	% a length of 30m and a PRECISION of 0.1m in both coordinates.
	LEN = 30;
	WIDTH = 30;
	PRECISION = 5;
	plan = ones(LEN*PRECISION+1, WIDTH*PRECISION+1, 'uint8'); % At first, all cells are unexplored.
    
	% Create a 2D mesh of points, stored in the vectors X and Y. 
	% Grid to store the points from the area that the hokuyo captor sees.
	DIST_BEAM = 5*PRECISION+2;
	[X, Y] = meshgrid(-DIST_BEAM:1:DIST_BEAM, -DIST_BEAM:1:DIST_BEAM);
	X = reshape(X, 1, []); % Make a vector of the matrix X.
	Y = reshape(Y, 1, []);

	% Initialise the state machine.
    if INITIAL_MAPPING
        step = 'mapping';
        fsm = 'start';
    else
        struct = load('start_plan.mat');
        plan = cell2mat(struct2cell(struct));
        struct = load('start_min_map.mat');
        min_map = cell2mat(struct2cell(struct));
        struct = load('start_max_map.mat');
        max_map = cell2mat(struct2cell(struct));
        step = 'mapping';
        fsm = 'new_destination';
    end

	% Make sure everything is settled before we start.
	pause(2);
	%% Start the demo.
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
		
		%% ******************* %%
		%        MAPPING        %
		%% ******************* %%
		if strcmp(step, 'mapping')
		
		
			%% ******************* %%
			%         START         %
			%% ******************* %%
			if strcmp(fsm, 'start')
				begin_starting = tic;
				if (INITIAL_MAPPING)
					%% Read data from the depth sensor, more often called the Hokuyo.
					% Determine the position of the Hokuyo with global coordinates (world reference frame).
					trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
					worldHokuyo1 = homtrans(trf, [h.hokuyo1Pos(1); h.hokuyo1Pos(2); h.hokuyo1Pos(3)]);
					worldHokuyo2 = homtrans(trf, [h.hokuyo2Pos(1); h.hokuyo2Pos(2); h.hokuyo2Pos(3)]);
					
					% Use the sensor to detect the visible points, within the world frame.
					[pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf);
					
					% Initialize map extremities.
					% Extremity of the map center on the initial youbot positio,.
					min_map = [round(PRECISION*youbotPos(1)) - LEN/2*PRECISION,...
						round(PRECISION*youbotPos(2)) - LEN/2*PRECISION];
					max_map = [round(PRECISION*youbotPos(1)) + LEN/2*PRECISION,...
						round(PRECISION*youbotPos(2)) + LEN/2*PRECISION];

					% Find the points inside the vision of the Hokuyo (center on
					% the youbot position).
					index_map = inpolygon(X, Y,...
						PRECISION*[worldHokuyo1(1), pts(1, :), worldHokuyo2(1)]- ...
						round(PRECISION*youbotPos(1)),...
						PRECISION*[worldHokuyo1(2), pts(2, :), worldHokuyo2(2)]- ...
						round(PRECISION*youbotPos(2)));
					
					% Find the points inside the vision of the Hokuyo on the
					% plan coordinate. (coordinate of X + youbot position) ->
					% gives the position in 10 centimeters, - min map -> to have
					% the minimum value = 0, + 1 -> indices in matlab begin to
					% 1
					new_visited = [transpose(X(index_map) - min_map(1) + ...
						round(PRECISION*youbotPos(1)) + 1)...
						transpose(Y(index_map) - min_map(2) + ...
						round(PRECISION*youbotPos(2)) + 1);...
						(round(PRECISION*worldHokuyo1(1))- min_map(1)+1)...
						(round(PRECISION*worldHokuyo1(2))- min_map(2)+1);...
						(round(PRECISION*youbotPos(1))- min_map(1)+1)...
						(round(PRECISION*youbotPos(2))- min_map(2)+1)];
					
					% Add visited area. (Plan center on the vision of the
					% Hokoyo.)
					for i = 1 : length(new_visited)
						plan(new_visited(i,1), new_visited(i,2)) = FREE_CELL;
					end
					
					% Vector containing the position of the obstacles. Unique
					% erases the identique rows.
					new_obstacle = (unique(round(PRECISION*[transpose(pts(1, contacts)) ...
						transpose(pts(2, contacts))]),'rows') - min_map + 1);
					
					% Add blocked area.
					for i = 1 : length(new_obstacle)
						plan(new_obstacle(i,1), new_obstacle(i,2)) = BLOCKED_CELL;
					end
					
					% begin the displacement by a rotation of180 degree
					trajectory_full = [round(PRECISION*youbotPos(1))- min_map(1)+1 round(PRECISION*youbotPos(2))- min_map(1)+1];
					pos_trajectory_full = 1;
					new_angle = youbotEuler(3)+pi;
					pos_update = [youbotPos(1) youbotPos(2)];
					angle_update = youbotEuler(3);
					% Plot of the total map.
                    figure(1)
					imagesc(plan)
					INITIAL_MAPPING = false;
				end
				rotateRightVel = angdiff(new_angle, youbotEuler(3));    % perform a 180 degrees rotation first
				if (abs(angdiff(new_angle, youbotEuler(3))) < 1 / 180 * pi) && (abs(angdiff(prevOrientation, youbotEuler(3)) < 0.1 / 180 * pi)) 
					rotateRightVel = 0;
					fsm = 'new_destination';	 % when rotation is done, compute the first target destination
				end
				end_starting = toc;
                % save the map obtain by the start
				save('start_plan.mat','plan'); 
				save('start_min_map.mat', 'min_map');
				save('start_max_map.mat', 'max_map');
			
			%% ******************* %%
			%        ROTATE         %
			%% ******************* %%	
			elseif strcmp (fsm, 'rotate')
				begin_rotate = tic
				rotateRightVel = angdiff(Angle, youbotEuler(3)); % perform the rotation
				
				if (abs(angdiff(new_angle, youbotEuler(3))) < 1 / 180 * pi) && (abs(angdiff(prevOrientation, youbotEuler(3)) < 0.1 / 180 * pi)) 
					rotateRightVel = 0;
					fsm = 'moving';     % ready to move to the next intermediate point of the trajectory
					
					%compute some of the distances used when 'moving'
					prev_position = [youbotPos(1) youbotPos(2)];    % initial position of the robot when it was rotating
					initial_dist = sqrt((trajectory(target,1) - (PRECISION*youbotPos(1)-minMap(1)+1))^2 +...     % inital distance compared to the next destination 
										(trajectory(target,2) - (PRECISION*youbotPos(2)-minMap(2)+1))^2);        % of the trajectory
					% target is the next step of the trajectory, the next
					% target to reach
									
					% but first check if the destination has been explored, if yes,
					% then look for a new destination                
					if (new_goal(1) == length(trajectory(:, 1)))

						if (new_goal(2) == length(trajectory(1,:)))     % case where target is in upper right corner of the map (max x, max y)
							if (    trajectory(new_goal(1), new_goal(2)) ~= 0 &&...     % if destination is explored and
									trajectory(new_goal(1), new_goal(2)-1) ~= 0 &&... % if case on the left is explored and
									trajectory(new_goal(1)-1, new_goal(2)) ~= 0 &&...   % if case below is explored and
									trajectory(new_goal(1)-1, new_goal(2)-1) ~= 0)      % if lower left case is explored then ok
								fsm = 'new_destination';                % search a new destination
							end

						else                                            % case where target is on top of the map (max y) but not on the right of the map (not max x)
							if (    trajectory(new_goal(1), new_goal(2)) ~= 0 &&...     % if destination is explored and
									trajectory(new_goal(1), new_goal(2)-1) ~= 0 &&... % if case on the left is explored and
									trajectory(new_goal(1)-1, new_goal(2)) ~= 0 &&...   % if case below is explored and
									trajectory(new_goal(1)-1, new_goal(2)-1) ~= 0 &&... % if lower left case is explored and
									trajectory(new_goal(1), new_goal(2)+1) ~= 0 &&...  % if case on right is explored and
									trajectory(new_goal(1)-1, new_goal(2)+1) ~= 0)      % if lower right case is explored then ok
								fsm = 'new_destination';                % search a new destination
							end
						end

					else     

						if (new_goal(2) == length(trajectory(1,:)))     % case where target is on extreme right of map but not on top
							 if (    trajectory(new_goal(1), new_goal(2)) ~= 0 &&...     % if destination is explored and
									trajectory(new_goal(1), new_goal(2)-1) ~= 0 &&... % if case on the left is explored and
									trajectory(new_goal(1)-1, new_goal(2)) ~= 0 &&...   % if case below is explored and
									trajectory(new_goal(1)-1, new_goal(2)-1) ~= 0 &&... % if lower left case is explored and
									trajectory(new_goal(1)+1, new_goal(2)) ~= 0 &&...  % if case on top is explored and
									trajectory(new_goal(1)+1, new_goal(2)-1) ~= 0)      % if upper left case is explored then ok
								fsm = 'new_destination';                % search a new destination
							 end

						else                                            % case where target is anywhere on the map
							 if (    trajectory(new_goal(1), new_goal(2)) ~= 0 &&...     % if destination is explored and
									trajectory(new_goal(1), new_goal(2)-1) ~= 0 &&... % if case on the left is explored and
									trajectory(new_goal(1)-1, new_goal(2)) ~= 0 &&...   % if case below is explored and
									trajectory(new_goal(1)-1, new_goal(2)-1) ~= 0 &&... % if lower left case is explored and
									trajectory(new_goal(1), new_goal(2)+1) ~= 0 &&...  % if case on right is explored and
									trajectory(new_goal(1)-1, new_goal(2)+1) ~= 0)      % if lower right case is explored then ok
								fsm = 'new_destination';                % search a new destination
							end
						end 
					end
				end
					
				prevOrientation = youbotEuler(3);   % put current orientation inside prevOrientation for use in the next computation cycle
				end_rotate = toc
				
				
			%% ******************* %%
			%          MOVE         %
			%% ******************* %%
			elseif strcmp (fsm, 'moving')
				begin_moving = tic
				distance = sqrt((trajectory(target, 1) - resol*youbotPos(1))^2 +...
								(trajectory(target, 2) - resol*youbotPos(2))^2);    % compute the distance wrt the target
				
				if initial_dist < 2*MAX_DISTANCE         % see if the complete distance is long enough to perform an acceleration/decceleration on MAX_DISTANCE
					half_dist = initial_dist/2;          % if not, acceleration/decceleration on half the distance to run

					% Acceleration.
					forwBackVel = -MAX_SPEED/MAX_DISTANCE * (inital_dist - distance);       % negative value of BackVel to go forward

					% Decceleration.
					if distance < half_distance          % when half the distance is crossed, deccelerate
						forwBackVel = -MAX_SPEED/MAX_DISTANCE * distance;
					end

				else
					if distance > initial_distance - MAX_DISTANCE
						forwBackVel = -MAX_SPEED/MAX_DISTANCE * (inital_dist - distance);   % set an increasing speed, depending on the crossed distance
					elseif distance < initial_distance - MAX_DISTANCE && distance > MAX_DISTANCE
						forwBackVel = -maxSpeed;
					else
						forwBackVel = -MAX_SPEED/MAX_DISTANCE * distance;                   % set a decreasing speed, depending on the remaining distance
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
							
				if (distance < 0.01) && (resol*sqrt((prevPosition(1)-youbotPos(1))^2 + ...
										(prevPosition(2)-youbotPos(2))^2) < 0.005)              % see if distance is close enough and speed low enough to stop
					forwBackVel = 0; % stop moving
					
					if (target == size(trajectory, 1)) 
						fsm = 'new_destination';        % final target reached, compute a new destination
					else
						target = target + 1;            % next target of the trajectory
						fsm = 'rotate';                 % rotate toward this target
						new_angle = atan(((PRECISION*youbotPos(1)-minMap(1)+1)-trajectory(target,1))/...
										(trajectory(target,2)-(PRECISION*youbotPos(2)-minMap(2)+1)));   % target has been increased, compute the angle wrt the nex target
						if (trajectory(target,2)-(PRECISION*youbotPos(2)-minMap(2)+1)) > 0           % used to correct the angle as it is computed 
							new_angle = new_angle + pi;                                                     % based on map coordinates, and not robot referential
						end
					end
				end
				
				prevPosition = [youbotPos(1) youbotPos(2)];         % save current position before next computation to see if speed is low enough when stopping the robot
				end_moving = toc
			
			
			%% ******************* %%
			%    NEW DESTINATION    %
			%% ******************* %%
			elseif strcmp(fsm, 'new_destination')
				begin_new_destination = tic
				% Define new goal
				% Explore the map to find accessible unexplored cells.
				boundaries = sparse([],[],[],double(max_map(1)-min_map(1)+1), double(max_map(2)-min_map(2)+1));
				for i = 2 : size(plan, 1)-1             
					for j = 2 : size(plan, 2)-1               
						% if visited cell
						if plan(i,j) == FREE_CELL
							%if in contact with unexplored cells
							if((plan(i-1,j) == UNEXPLORED_CELL)||(plan(i+1,j) == UNEXPLORED_CELL)|| ...
									(plan(i,j-1) == UNEXPLORED_CELL)||(plan(i,j+1) == UNEXPLORED_CELL))
								boundaries = boundaries + ...
									sparse(double(i),double(j), 1, double(max_map(1)-min_map(1)+1),...
									double(max_map(2)-min_map(2)+1));
							end
						end
					end
				end
				
				% extract rows and colums from boundaries
				[pos_row, pos_col] = find(boundaries == 1);
				
				% If the map is completely explored, navigation is finished.
				if isempty(pos_row)
					fsm = 'finished';
				else
					new_start = [round(PRECISION*(youbotPos(1)))-min_map(1)+1, round(PRECISION*(youbotPos(2)))-min_map(2)+1];
					
					% find destination with max distance for the first
					% trajectory_full (to extend the map a maximum at the begining)
					% and min for the others.
					if FIRST_TRAJ
						index_goal = find(hypot(pos_row-new_start(1), pos_col-new_start(2))==...
							max(hypot( pos_row-new_start(1), pos_col-new_start(2))),1,'first');
						FIRST_TRAJ = false;
					else
						index_goal = find(hypot(pos_row-new_start(1), pos_col-new_start(2))==...
							min(hypot( pos_row-new_start(1), pos_col-new_start(2))),1,'first');
					end
				 
					new_goal = [pos_col(index_goal) pos_row(index_goal)]
					% D* algorithm for planning
					%figure;
%                     Dstar_plan = plan;
%                     Dstar_plan(find(Dstar_plan == BLOCKED_CELL)) = UNEXPLORED_CELL;
					ds = Dstar(plan)    % create navigation object
					ds.plan(new_goal);   % create plan for specified goal
					trajectory_full = ds.path(flip(NewStart))% animate path from this start location
					%trajectory_full = ds.query(start);      

					% to have x than y 
					trajectory_full = flip(trajectory_full,2); 
					if size(trajectory_full,1)>1
						
						trajectory2 = zeros(size(trajectory_full,1), 1);
						% Check if the next position in the trajectory_full is in a straight line. if it is not
						% put the given element in trajectory2 to 1. (rotation needed)
						if ((trajectory_full(1,1)- new_start(1)) ~= (trajectory_full(2,1)-trajectory_full(1,1)))||...
								((trajectory_full(1,2)- new_start(2)) ~= (trajectory_full(2,2)-trajectory_full(1,2)))
								trajectory2(1) = 1;
						end
						for i = 2: length(trajectory_full)-1
							if ((trajectory_full(i,1)-trajectory_full(i-1,1)) ~= (trajectory_full(i+1,1)-trajectory_full(i,1)))||...
								((trajectory_full(i,2)-trajectory_full(i-1,2)) ~= (trajectory_full(i+1,2)-trajectory_full(i,2)))
								trajectory2(i) = 1;
							end
						end
						% Add the before last element to 1. (To rotate before arriving to the goal position.)
						trajectory2(end-1) = 1;
						trajectory2(end) = 0;
						trajectory = trajectory_full(trajectory2 == 1, :);
					end
					
					postrajectory = 1;
					% !!!TOCHECK!!! Operation use to change from the world referentiel to the Hokuyo referentiel.
					% Try to do this with trox
					new_angle = atan(((PRECISION*youbotPos(1)-min_map(1)+1)-trajectory(1,1))/...
						(trajectory(1,2)) - (PRECISION*youbotPos(2)-min_map(2)+1));
					if (trajectory(1,2)) - (PRECISION*youbotPos(2)-min_map(2)+1) > 0
                            new_angle = new_angle + pi;
                    end   
					fsm = 'rotate';
				end        
			stop_new_destination = toc
			
			
			%% ******************* %%
			%      END MAPPING      %
			%% ******************* %%
			elseif  strcmp(fsm, 'finished')  
				pause(1);
				step = 'finished';
				 % save the map obtain by the navigation
				save('plan.mat','plan'); 
				save('min_map.mat', 'min_map');
				save('max_map.mat', 'max_map');
				fprintf('Switching to step: %s\n', step);
				
				
			%% ******************* %%
			%         ERROR         %
			%% ******************* %%
			else
				fsm = 'finished';
				error('Unknown state %s.', fsm);
			end
			
			%% ******************* %%
			%       MAP UPDATE      %
			%% ******************* %%
			% Update afer a distance of 1m or a rotation of 30 degree
			if (sqrt((youbotPos(1)-pos_update(1))^2 + (youbotPos(2)-pos_update(2))^2) >= 1)...
				||(abs(angdiff(angle_update - youbotEuler(3))) > (pi/6))
				begin_update_map = tic
				pos_update = [youbotPos(1) youbotPos(2)];
				angle_update = youbotEuler(3);
				%% Read data from the depth sensor, more often called the Hokuyo.
				% Determine the position of the Hokuyo with global coordinates (world reference frame).
				trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
				worldHokuyo1 = homtrans(trf, [h.hokuyo1Pos(1); h.hokuyo1Pos(2); h.hokuyo1Pos(3)]);
				worldHokuyo2 = homtrans(trf, [h.hokuyo2Pos(1); h.hokuyo2Pos(2); h.hokuyo2Pos(3)]);
				
				% Use the sensor to detect the visible points, within the world frame. 
				[pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf);
				%update map extremity
				min_map_new = round(PRECISION*[min([worldHokuyo1(1), pts(1, :), worldHokuyo2(1)])...
					min([worldHokuyo1(2), pts(2, :), worldHokuyo2(2)])]) - 1;
				max_map_new = round(PRECISION*[max([worldHokuyo1(1), pts(1, :), worldHokuyo2(1)])...
					max([worldHokuyo1(2), pts(2, :), worldHokuyo1(2)])]) + 1;
				% if map need to be extended
				% modify min x
				if (min_map_new(1) < min_map(1))
					%update plan
					min_map_tmp = [min_map_new(1)-5*PRECISION min_map(2)];
					%plan = horzcat(zeros(size(plan,1), min_map(2) - min_map_tmp(2), 'uint8'),...
					%plan, zeros(size(plan,1), max_map_new(2) - max_map(2), 'uint8'));
					plan = vertcat(zeros(min_map(1) - min_map_tmp(1), size(plan,2), 'uint8'),...
					plan, zeros(max_map_new(1) - max_map(1), size(plan,2), 'uint8'));
					% update trajectory_full
					trajectory_full = trajectory_full + min_map - min_map_tmp;
					% update min_map
					min_map = min_map_tmp;
				end
				% modify min y
				if (min_map_new(2) < min_map(2))
					%update plan
					min_map_tmp = [min_map(1) min_map_new(2) - 5*PRECISION];
					plan = horzcat(zeros(size(plan,1), min_map(2) - min_map_tmp(2), 'uint8'),...
					plan, zeros(size(plan,1), max_map_new(2) - max_map(2), 'uint8'));
					%plan = vertcat(zeros(min_map(1) - min_map_tmp(1), size(plan,2), 'uint8'),...
					%plan, zeros(max_map_new(1) - max_map(1), size(plan,2), 'uint8'));
					% update trajectory_full
					trajectory_full = trajectory_full + min_map - min_map_tmp;
					% update min_map
					min_map = min_map_tmp;
				end
				% modify max x
				if (max_map_new(1) > max_map(1))
					%update plan
					max_map_tmp = [max_map_new(1)+5*PRECISION max_map(2)];
					%plan = horzcat(zeros(size(plan,1), min_map(2) - min_map_new(2), 'uint8'),...
					%plan, zeros(size(plan,1), max_map_tmp(2) - max_map(2), 'uint8'));
					plan = vertcat(zeros(min_map(1) - min_map_new(1), size(plan,2), 'uint8'),...
					plan, zeros(max_map_tmp(1) - max_map(1), size(plan,2), 'uint8'));
					% update min_map
					max_map = max_map_tmp;
				end
				% modify max y
				if (max_map_new(2) > max_map(2))
											%update plan
					max_map_tmp = [max_map(1) max_map_new(2)+5*PRECISION];
					plan = horzcat(zeros(size(plan,1), min_map(2) - min_map_new(2), 'uint8'),...
					plan, zeros(size(plan,1), max_map_tmp(2) - max_map(2), 'uint8'));
					%plan = vertcat(zeros(min_map(1) - min_map_new(1), size(plan,2), 'uint8'),...
					%plan, zeros(max_map_tmp(1) - max_map(1), size(plan,2), 'uint8'));
					% update min_map
					max_map = max_map_tmp;
				end
				
				% update walls
				new_obstacle = (unique(round(PRECISION*[transpose(pts(1, contacts)) ...
				transpose(pts(2, contacts))]),'rows') - min_map + 1);
				% Add blocked area.
				for i = 1 : length(new_obstacle)
					plan(new_obstacle(i,1), new_obstacle(i,2)) = BLOCKED_CELL;
				end
				
				%update visited area
				% Find the points inside the vision of the Hokuyo (center on
				% the youbot position).
				index_map = inpolygon(X, Y,...
					PRECISION*[worldHokuyo1(1), pts(1, :), worldHokuyo2(1)]- ...
					round(PRECISION*youbotPos(1)),...
					PRECISION*[worldHokuyo1(2), pts(2, :), worldHokuyo2(2)]- ...
					round(PRECISION*youbotPos(2)));
				
				% Find the points inside the vision of the Hokuyo on the
				% plan coordinate. (coordinate of X + youbot position) ->
				% gives the position in 10 centimeters, - min map -> to have
				% the minimum value = 0, + 1 -> indices in matlab begin to
				% 1
				new_visited = [transpose(X(index_map) - min_map(1) + ...
					round(PRECISION*youbotPos(1)) + 1)...
					transpose(Y(index_map) - min_map(2) + ...
					round(PRECISION*youbotPos(2)) + 1);...
					(round(PRECISION*worldHokuyo1(1))- min_map(1)+1)...
					(round(PRECISION*worldHokuyo1(2))- min_map(2)+1);...
					(round(PRECISION*youbotPos(1))- min_map(1)+1)...
					(round(PRECISION*youbotPos(2))- min_map(2)+1)];
				
				% Add visited area. (Plan center on the vision of the
				% Hokoyo.)
				for i = 1 : length(new_visited)
					if plan(new_visited(i,1), new_visited(i,2)) == UNEXPLORED_CELL
						plan(new_visited(i,1), new_visited(i,2)) = FREE_CELL;
					end
				end
				% Plot of the total map.
				figure(1)
				imagesc(plan)
				end_update_map = toc
			end
		%% ******************* %%
		%          END          %
		%% ******************* %%
		elseif strcmp(step, 'finished')
			pause(3);
			break;
		end
				%{
				% Perform a move, a rotate or check the map.
				if int_mapping
					fsm = 'motion';
					int_mapping = false;
				elseif rotation_start
					fsm = 'rotate';
				elseif Start
					fsm = 'starting';
				else
					fsm = 'check_end_mapping';
				end
			else strcmp(fsm, 'rotate')
				%%
				%compute the next position X and Y coordinates in the world
				%reference
				next_position_x = (single(new_traj(indice).x) - len/2)/10 ;
				next_position_y = (single(new_traj(indice).y) - WIDTH/2)/10 ;
				
				% Apply the initial rotation
				if rotation_start
					if abs(youbotPos(1) - next_position_x) < abs(youbotPos(2) - next_position_y)        % compare the difference between the coordinates X and Y of the next position and the robot
						rotate_y = true;                                                                %If the highest is the Y diff, movement along the Y axis so angle is pi or 0
					else
						rotate_y = false;                                                               %Else, movement along the x axis and angle is pi/2 or -pi/2
					end
					rotation_start = false;
				end
				
				% Apply a rotation
				if rotate_y                                         %Compare the Y coordinate of the next position and the robot
					if next_position_y - youbotPos(2) > 0           %If the difference is positive, direct the robot towards the increasing y coordinates
						angle = pi;
					else
						angle = 0;                                  %Else, toward the decreasing Y coordinates
					end
				else                                                %Same for the X coordinate
					if next_position_x - youbotPos(1) > 0
						angle = pi/2;
					else
						angle = -pi/2;
					end
				end
				
				%         display_rot = sprintf('Target angle = %f, current angle = %f, angle difference = %f and next target is in x = %d and y = %d',...
				%             angle, youbotEuler(3),angdiff(angle, youbotEuler(3)), new_traj(indice).y, new_traj(indice).x );
				%         disp(display_rot);
				
				% The rotation velocity depends on the difference between the current angle and the target.
				rotateRightVel = angdiff(angle, youbotEuler(3));
				% When the rotation is done (with a sufficiently high PRECISION), perform a mapping and move on to the next state.
				if (abs(angdiff(angle, youbotEuler(3))) < .05 / 180 * pi)
					rotateRightVel = 0;
					fsm = 'mapping';
					storing = true;
					int_mapping = true;
				end
			
			elseif strcmp(fsm, 'motion')
				%%
				% The further the robot, the faster it drives.
				% Determine if the move is in the y direction or x direction.
				if storing
					display_indice = sprintf('Indice = %d, length new_traj = %d', indice, length(new_traj));
					disp(display_indice);
					if abs(youbotPos(1) - next_position_x) < abs(youbotPos(2) - next_position_y)        % compare the difference between the coordinates X and Y of the next position and the robot
						move_y = true;                                                                  %If the highest is the Y diff, movement along the Y axis
					else
						move_y = false;                                                                 %Else, movement along the x axis
					end
					
					% Check if the new target is reachable.
					for i = -3:3
						for j = -3:3
							if plan(new_traj(indice).x + i, new_traj(indice).y + j) == 2
								disp('Current trajectory_full is wrong, compute a new trajectory_full');
								fsm = 'check_end_mapping';
								restart_traj = true;
								indice = 1;
								break;
							end
						end
						if restart_traj
							% error('traj restarted')
							break;
						end
					end
				end
				
				% Apply a motion in x or y direction.
				if move_y
					if next_position_y > youbotPos(2)
						distance = next_position_y - youbotPos(2);
					else
						distance = -(next_position_y - youbotPos(2));
					end
				else
					if next_position_x > youbotPos(1)
						distance = next_position_x - youbotPos(1);
					else
						distance = -(next_position_x - youbotPos(1));
					end
				end
				
				if ~restart_traj
					forwBackVel = -distance;
				end
				
				%Store the initial distance between the robot and the next position
				if storing
					storing = false;
					initial_distance = distance;
					if initial_distance > 4
						int_mapping = true;
					end
				end
				
				%If the distance increases, the robot points toward the wrong
				%direction, start a new rotation to correct it
				if (distance - initial_distance)/initial_distance > 1.2
					fsm = 'rotate';
					rotation_start = true;
				end
				
				%         display_move = sprintf('The remaining distance = %f, indice = %d, length new_traj = %d, next target is in x = %d, y = %d',...
				%             distance, indice, length(new_traj), new_traj(indice).y, new_traj(indice).x );
				%         disp(display_move);
				
				% If the distance between the target and the robot reach 50% of the
				% initial distance, performs a mapping:
				if ( ((initial_distance - distance)/initial_distance) > 1/2 && int_mapping)
					fsm = 'mapping';
					disp('Intermediate mapping performed')
				end
				% If the robot is sufficiently close and its speed is sufficiently
				% low, stop it and check the map
				if indice == length(new_traj) && distance < 0.5
					forwBackVel = 0;
					fsm = 'mapping';
					indice = 1;
				elseif (distance < .05) && indice ~= length(new_traj)
					forwBackVel = 0;
					fsm = 'mapping';
					rotation_start = true;
					indice = indice + 1;
					if indice > 1
						display_reached_tar = ['Target ', indice - 1 ' reached, going to target ', indice ];
						disp(display_reached_tar);
					end
				end
				
			elseif strcmp(fsm, 'starting')
				%%
				if initial_move
					initial_pos_y = youbotPos(2);
					youbot_in_pos_plan_x = uint8(10*youbotPos(1)+ len/2);
					youbot_in_pos_plan_y = uint8(10*youbotPos(2)+ WIDTH/2);
					for i = 0:11
						if plan(youbot_in_pos_plan_x, youbot_in_pos_plan_y  - 0.1*i) == 2
							backwards = true;
						end
					end
					initial_move = false;
				end
				
				if backwards == true
					distance = abs( youbotPos(2) - initial_pos_y - 1 );
					forwBackVel = distance;
				elseif backwards == false
					distance = abs( youbotPos(2) - initial_pos_y + 1 );
					forwBackVel = -distance;
				end
				
				if ((distance < .001) && rotate == false)
					forwBackVel = 0;
					fsm = 'mapping';
					rotate = true;
				end
				
				if rotate
					rotateRightVel = angdiff(angl, youbotEuler(3));
					% When the rotation is done (with a sufficiently high PRECISION), perform a mapping and move on to the next state.
					if (abs(angdiff(angl, youbotEuler(3))) < .1 / 180 * pi) && ...
							(abs(angdiff(prevOrientation, youbotEuler(3))) < .01 / 180 * pi)
						rotateRightVel = 0;
						fsm = 'mapping';
						Start = false;
						rotate = false;
					end
					prevOrientation = youbotEuler(3);
				end
				
			elseif strcmp(fsm, 'check_end_mapping')
				%%
				% check if the map is complete. To check if the map is complete,
				% we check if there exist a contour of BLOCKED_CELL.
				youbot_pos_plan_x = uint8(10*youbotPos(1)+ len/2);
				youbot_pos_plan_y = uint8(10*youbotPos(2)+ WIDTH/2);
				% traj_full contains all the steps needed to reach the target.
				[complete, traj_full, distance_check_map] = check_map(plan, youbot_pos_plan_x, youbot_pos_plan_y, distance_check_map)
				toc
				% new_traj array contains the steps to reach the target following straight lines.
				new_traj = [];
				L = length(traj_full);
				x_previous = traj_full(1).x;
				x_current = traj_full(2).x;
				if x_previous == x_current
					x_direction = true;
				else
					x_direction = false;
				end
				for i = 3:1:L
					x_previous = x_current;
					x_current = traj_full(i).x;
					if x_current == x_previous
						if ~x_direction
							new_traj  = [new_traj, traj_full(i-1)];
						end
						x_direction = true;
					else
						if x_direction
							new_traj = [new_traj, traj_full(i-1)];
						end
						x_direction = false;
					end
				end
				new_traj = [new_traj, traj_full(L)];
				% Start a rotation.
				rotation_start = true;
				restart_traj = false;
				fsm = 'rotate';
				% if the map is complete we finish the mapping.
				if complete
					fsm = 'finished';
				end
				
			elseif strcmp(fsm, 'finished')
				%% Demo done: exit the function.
				% With this representation, the contour function can be used to draw the map.
				figure(1);
				contour(plan);
				pause(3);
				break;
			else
				error('Unknown state %s.', fsm);
			end
		%}
		% Update wheel velocities using the global values (whatever the state is).
		h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);
		
		% Make sure that we do not go faster than the physics simulation (each iteration must take roughly 50 ms).
		elapsed = toc;
		timeleft = timestep - elapsed;
		if timeleft > 0
			pause(min(timeleft, .01));
		end
	end
end