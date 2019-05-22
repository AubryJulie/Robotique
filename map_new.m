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
	% Start = true;           % First go to starting state after first end_map_checking
	% int_mapping = false;    % When half the distance is crossed when in 'motion', perform an intermediate mapping
	% rotate = false;         % Determines if the robot performs the initial rotation or motion
	% initial_move = true;    % Go forwards or backwards by one meter before performing the initial rotation
	% backwards = false;      % Determines if the robot goes forward or backwards
	% rotation_start = false;
	angl = pi;
	indice = 1;
	distance_check_map = 5;   % 5 meters (use in check map)

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
	unexplored_cell = 0;
	free_cell = 1;
	blocked_cell = 2;
	% plan is a vector containing the map of the explorated house. The plan as
	% a length of 30m and a precision of 0.1m in both coordinates.
	len = 30;
	width = 30;
	precision = 10;
	plan = zeros(len*10+1, width*10+1, 'uint8'); % At first, all cells are unexplored.

	% Create a 2D mesh of points, stored in the vectors X and Y. 
	% Grid to store the points from the area that the hokuyo captor sees.
	distBeam = 5*precision+2;
	[X, Y] = meshgrid(-distBeam:1:distBeam, -distBeam:1:distBeam);
	X = reshape(X, 1, []); % Make a vector of the matrix X.
	Y = reshape(Y, 1, []);

	% Initialise the state machine.
	step = 'mapping';
	fsm = 'start';

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
		
		%% MAPPING.
		if strcmp(step, 'mapping')
			if strcmp(fsm, 'start')
				%% Read data from the depth sensor, more often called the Hokuyo.
				% Determine the position of the Hokuyo with global coordinates (world reference frame).
				trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
				worldHokuyo1 = homtrans(trf, [h.hokuyo1Pos(1); h.hokuyo1Pos(2); h.hokuyo1Pos(3)]);
				worldHokuyo2 = homtrans(trf, [h.hokuyo2Pos(1); h.hokuyo2Pos(2); h.hokuyo2Pos(3)]);
				
				% Use the sensor to detect the visible points, within the world frame.
				[pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf);
				
				% Initialize map extremities.
                % Extremity in the Hokuyo visions
				min_map_new = floor(precision*[min([worldHokuyo1(1), pts(1, :), worldHokuyo2(1)])...
					min([worldHokuyo1(2), pts(2, :), worldHokuyo2(2)])]);
				max_map_new = ceil(precision*[max([worldHokuyo1(1), pts(1, :), worldHokuyo2(1)])...
					max([worldHokuyo1(2), pts(2, :), worldHokuyo2(2)])]);
                
                % Extremity of the map center on the Hokuyo vision.
				min_map = [round((max_map_new(1) + min_map_new(1))/2) - len/2*precision,...
					round((max_map_new(2) + min_map_new(2))/2) - len/2*precision];
				max_map = [round((max_map_new(1) + min_map_new(1))/2) + len/2*precision,...
					round((max_map_new(2) + min_map_new(2))/2) + len/2*precision];

                % Find the points inside the vision of the Hokuyo (center on
                % the youbot position).
				index_map = inpolygon(X, Y,...
					precision*[worldHokuyo1(1), pts(1, :), worldHokuyo2(1)]- ...
					round(precision*youbotPos(1)),...
					precision*[worldHokuyo1(2), pts(2, :), worldHokuyo2(2)]- ...
					round(precision*youbotPos(2)));
                
				% Find the points inside the vision of the Hokuyo on the
				% plan coordinate. (coordinate of X + youbot position) ->
				% gives the position in 10 centimeters, - min map -> to have
				% the minimum value = 0, + 1 -> indices in matlab begin to
				% 1
				new_visited = [transpose(X(index_map) - min_map(1) + ...
					round(precision*youbotPos(1)) + 1)...
					transpose(Y(index_map) - min_map(2) + ...
					round(precision*youbotPos(2)) + 1);...
					(round(precision*worldHokuyo1(1))- min_map(1)+1)...
					(round(precision*worldHokuyo1(2))- min_map(2)+1);...
					(round(precision*youbotPos(1))- min_map(1)+1)...
					(round(precision*youbotPos(2))- min_map(2)+1)];
                
				% Add visited area. (Plan center on the vision of the
				% Hokoyo.)
				for i = 1 : length(new_visited)
					plan(new_visited(i,1), new_visited(i,2)) = free_cell;
                end
                
				% Vector containing the position of the obstacles. Unique
				% erases the identique row.
				new_obstacle = (unique(round(precision*[transpose(pts(1, contacts)) ...
                    transpose(pts(2, contacts))]),'rows') - min_map + 1);
				
				% Add blocked area.
				for i = 1 : length(new_obstacle)
					plan(new_obstacle(i,1), new_obstacle(i,2)) = blocked_cell;
				end
				
				fsm = 'rotate';
				step = 'finished';
				% Plot of the total map.
				figure;
				imagesc(plan)
			end
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
				next_position_y = (single(new_traj(indice).y) - width/2)/10 ;
				
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
				% When the rotation is done (with a sufficiently high precision), perform a mapping and move on to the next state.
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
								disp('Current trajectory is wrong, compute a new trajectory');
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
					youbot_in_pos_plan_y = uint8(10*youbotPos(2)+ width/2);
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
					% When the rotation is done (with a sufficiently high precision), perform a mapping and move on to the next state.
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
				% we check if there exist a contour of blocked_cell.
				youbot_pos_plan_x = uint8(10*youbotPos(1)+ len/2);
				youbot_pos_plan_y = uint8(10*youbotPos(2)+ width/2);
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