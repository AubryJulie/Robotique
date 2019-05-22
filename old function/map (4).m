function map()
% map: creation of the map

% Initiate the connection to the simulator.
%addpath('C:\Users\MonOrdi\Documents\trs\youbot');
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

%% Preset values for the demo.
disp('Starting robot');

% Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels.
% They are adapted at each iteration by the code.
forwBackVel = 0;        % Move straight ahead.
rightVel = 0;           % Go sideways.
rotateRightVel = 0;     % Rotate.
prevOrientation = 0;    % Previous angle to goal (easy way to have a condition on the robot's angular speed).
prevPosition = 0;       % Previous distance to goal (easy way to have a condition on the robot's speed).
Start = true;           % First go to starting state after first end_map_checking
int_mapping = false;    % When half the distance is crossed when in 'motion', perform an intermediate mapping
rotate = false;         % Determines if the robot performs the initial rotation or motion
initial_move = true;    % Go forwards or backwards by one meter before performing the initial rotation
backwards = false;      % Determines if the robot goes forward or backwards
angl = pi;
indice = 1;

res = vrep.simxPauseCommunication(id, false);
vrchk(vrep, res);

% Initialise the state machine.
fsm = 'mapping';


% Initialise the map.
unexplored_cell = 0;
free_cell = 1;
blocked_cell = 2;
% plan is a vector containing the map of the explorated house. The plan as
% a length of 30m and a precision of 0.1m in both coordinates.
len = 300;
width = 300;
plan = zeros(len, width, 'uint8'); % At first, all cells are unexplored.

% % Create a 2D mesh of points, stored in the vectors X and Y. This will be
% % used to display the area the robot can see.
% [X, Y] = meshgrid(-15:.1:15, -15:.1:15);
% X = reshape(X, 1, []); % Make a vector of the matrix X.
% Y = reshape(Y, 1, []);
% Create a 2D mesh of points, stored in the vectors X and Y. This will be
% used to display the area the robot can see. Use X and Y of higher
% dimension than the plan in orther to check if we need to resize the plan.
[X, Y] = meshgrid(-(len/20+5):.1:(len/20+5), -(width/20+5):.1:(width/20+5));
X = reshape(X, 1, []); % Make a vector of the matrix X.
Y = reshape(Y, 1, []);

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
    
        %% Apply the state machine.
    if strcmp(fsm, 'rotate')
        %% 
        %compute the distance and the angle difference with the target
        next_position_x = (single(new_traj(indice).x) - len/2)/10 ;
        next_position_y = (single(new_traj(indice).y) - width/2)/10 ;
                
        if rotation_start
            if abs(youbotPos(1) - next_position_x) < 0.05
            	rotate_y = true;
            elseif abs(youbotPos(2) - next_position_y) < 0.05
                rotate_y = false;
            else
                error('error in rotate');
            end
            rotation_start = false;
        end
        
        if rotate_y
            if next_position_y - youbotPos(2) > 0
                angle = pi;
            else
                angle = 0;
            end
        else
            if next_position_x - youbotPos(1) > 0
                angle = pi/2;
            else
                angle = -pi/2;
            end
        end

        % The rotation velocity depends on the difference between the current angle and the target.
        rotateRightVel = angdiff(angle, youbotEuler(3));
        % When the rotation is done (with a sufficiently high precision), move on to the next state.
        if (abs(angdiff(angle, youbotEuler(3))) < .1 / 180 * pi) %&& ...
              %  (abs(angdiff(angle, youbotEuler(3))) < .01 / 180 * pi)
            rotateRightVel = 0;
            fsm = 'motion';
            storing = true;
        end
        
    elseif strcmp(fsm, 'motion')
        %%
        % The further the robot, the faster it drives.
        
        if storing
            if abs(youbotPos(1) - next_position_x) < 0.05
                move_y = true;
            elseif abs(youbotPos(2) - next_position_y) < 0.05
                move_y = false;
            else
                error('Error in motion');
            end
        end
        
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

        forwBackVel = -distance;
        %
        if storing
            storing = false;
            initial_distance = distance;
            int_mapping = true;
        end
        % If the distance between the target and the robot reach 50% of the
        % initial distance, perform a mapping:
        if ( ((initial_distance - distance)/initial_distance) < 1/2 && int_mapping)
            fsm = 'mapping';
        end
        % If the robot is sufficiently close and its speed is sufficiently
        % low, stop it and check the map
        if (distance < .001) % && (abs(youbotPos(1) - prevPosition) < .001)
            forwBackVel = 0;
            if indice == length(new_traj)
                fsm = 'mapping';
                indice = 1;
            else
                fsm = 'rotate';
                rotation_start = true;
                indice = indice + 1;
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
            
        if ((distance < .001) && rotate == false) % && (abs(youbotPos(1) - prevPosition) < .001)
            forwBackVel = 0;
            fsm = 'mapping';
            rotate = true;
        end 
        
        if rotate
            rotateRightVel = angdiff(angl, youbotEuler(3));
            % When the rotation is done (with a sufficiently high precision), move on to the next state.
            if (abs(angdiff(angl, youbotEuler(3))) < .1 / 180 * pi) && ...
                    (abs(angdiff(prevOrientation, youbotEuler(3))) < .01 / 180 * pi)
                rotateRightVel = 0;
                fsm = 'mapping';
                Start = false;
                rotate = false;
            end
            prevOrientation = youbotEuler(3);
        end
                
    elseif strcmp(fsm, 'mapping')
        %% Read data from the depth sensor, more often called the Hokuyo.
        % Determine the position of the Hokuyo with global coordinates (world reference frame).
        trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
        worldHokuyo1 = homtrans(trf, [h.hokuyo1Pos(1); h.hokuyo1Pos(2); h.hokuyo1Pos(3)]);
        worldHokuyo2 = homtrans(trf, [h.hokuyo2Pos(1); h.hokuyo2Pos(2); h.hokuyo2Pos(3)]);
        
        % Use the sensor to detect the visible points, within the world frame.
        [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf);
        
        % Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo (it returns the area that
        % is visible, but the visualisation draws a series of points that are within this visible area).
        % Complete the plan.
        in = inpolygon(X, Y,...
            [worldHokuyo1(1), pts(1, :), worldHokuyo2(1)],...
            [worldHokuyo1(2), pts(2, :), worldHokuyo2(2)]);
        new_X = X(in);
        new_Y = Y(in);
        l_X = length(new_X);
        tic
        for i = 1:l_X
            if uint8(10*new_X(i)+len/2) > len || uint8(10*new_X(i)+len/2) < 0
                [plan, len] = extend_x(plan);
                [X, Y] = meshgrid(-(len/20+5):.1:(len/20+5), -(width/20+5):.1:(width/20+5));
                X = reshape(X, 1, []); % Make a vector of the matrix X.
                Y = reshape(Y, 1, []);
            end
            if uint8(10*new_Y(i)+width/2) > width || uint8(10*new_Y(i)+width/2) < 0
                [plan, width] = extend_y(plan);
                [X, Y] = meshgrid(-(len/20+5):.1:(len/20+5), -(width/20+5):.1:(width/20+5));
                X = reshape(X, 1, []); % Make a vector of the matrix X.
                Y = reshape(Y, 1, []);
            end
            plan(uint8(10*new_X(i)+len/2),uint8(10*new_Y(i)+width/2)) = free_cell;
        end
        toc
        point = pts(:,contacts);
        l_p = length(point);
        for i=1:l_p
            x = point(1,i);
            y = point(2,i);
            if uint8(10*(x)+len/2) > len || uint8(10*(x)+len/2) < 0
                [plan, len] = extend_x(plan);
            end
            if uint8(10*(y)+width/2) > width || uint8(10*(y)+width/2) < 0
                [plan, width] = extend_y(plan);
            end
            plan(uint8(10*(x)+len/2), uint8(10*(y)+width/2)) = blocked_cell;
        end
        figure(1)
        contour(plan);
        if int_mapping
            fsm = 'motion';
            int_mapping = false;
        else
            fsm = 'check_end_mapping';
        end
        
    
    elseif strcmp(fsm, 'check_end_mapping')
        %%       
        % check if the map is complete. To check if the map is complete,
        % we check if there exist a contour of blocked_cell
        fsm = 'trajectory';
        if Start
            fsm = 'starting';
        end
        youbot_pos_plan_x = uint8(10*youbotPos(1)+ len/2);
        youbot_pos_plan_y = uint8(10*youbotPos(2)+ width/2);
        [complete, next_pos_x, next_pos_y] = check_map(plan, youbot_pos_plan_x, youbot_pos_plan_y);
        % transformation to the world coordinates
%         next_position_x = (single(next_pos_x) - len/2)/10 ;
%         next_position_y = (single(next_pos_y) - width/2)/10 ;
        if complete
            fsm = 'finished';
        end
    elseif strcmp(fsm, 'trajectory')
        %%
        [find, traj_full] = trajectory(plan, youbot_pos_plan_x, youbot_pos_plan_y, next_pos_x, next_pos_y);
        toc
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
        for i = 1:length(new_traj)
            new_traj(i).x
            new_traj(i).y
            plan(new_traj(i).x, new_traj(i).y) = 3;
        end
        next_pos_x
        next_pos_y
        plan(next_pos_x, next_pos_y) = 4;
        figure(1)
        contour(plan);
        rotation_start = true;
        fsm = 'rotate';

    
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