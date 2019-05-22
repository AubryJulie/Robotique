function map()
% map: creation of the map

% Initiate the connection to the simulator.
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
forwBackVel = 0; % Move straight ahead.
rightVel = 0; % Go sideways.
rotateRightVel = 0; % Rotate.
prevOrientation = 0; % Previous angle to goal (easy way to have a condition on the robot's angular speed).
prevPosition = 0; % Previous distance to goal (easy way to have a condition on the robot's speed).

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
    if strcmp(fsm, 'mapping')
        % Read data from the depth sensor, more often called the Hokuyo.
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
        fsm = 'check_end_mapping';
        angl = pi;
        %% Apply the state machine.
    elseif strcmp(fsm, 'rotate')
        %% First, rotate the robot to go to one table.
        % The rotation velocity depends on the difference between the current angle and the target.
        rotateRightVel = angdiff(angl, youbotEuler(3));
        % When the rotation is done (with a sufficiently high precision), move on to the next state.
        if (abs(angdiff(angl, youbotEuler(3))) < .1 / 180 * pi) && ...
                (abs(angdiff(prevOrientation, youbotEuler(3))) < .01 / 180 * pi)
            rotateRightVel = 0;
            fsm = 'mapping';
        end
        prevOrientation = youbotEuler(3);
    elseif strcmp(fsm, 'check_end_mapping')
        
        % check if the map is complete. To check if the map is complete,
        % we check if there exist a contour of blocked_cell
        fsm = 'move';
        youbot_pos_plan_x = uint8(10*youbotPos(1)+len/2);
        youbot_pos_plan_y = uint8(10*youbotPos(2)+width/2);
        [complete, next_pos_x, next_pos_y] = check_map(plan, youbot_pos_plan_x, youbot_pos_plan_y);
        fsm = 'trajectory';
        %         if complete
        %             fsm = 'finished';
        %         end
    elseif strcmp(fsm, 'trajectory')
        [find, traj] = trajectory(plan, youbot_pos_plan_x, ...
            youbot_pos_plan_y, next_pos_x, next_pos_y)
        fsm = 'finished';
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