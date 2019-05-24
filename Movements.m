initial_mapping = 1;
max_speed = 40; % determine a maximal speed for the robot
max_distance = 50;  % acceleration over 50 meters

if strcmp (fsm, 'start')
    
    if (initial_mapping == 1)
        
    % perform start initialization
    % ...
    % when initialization is finished:
    
    initial_mapping = 0;
    
    end
    
    rotateRightVel = angdiff(new_angle, youbotEuler(3));    % perform a 180 degrees rotation first
    if (abs(angdiff(new_angle, youbotEuler(3))) < 1 / 180 * pi) && (abs(angdiff(prevOrientation, youbotEuler(3)) < 0.1 / 180 * pi)) 
            rotateRightVel = 0;
            fsm = 'new_destination';     % when rotation is done, compute the first target destination
    end 


elseif strcmp (fsm, 'rotate')
    
    rotateRightVel = angdiff(Angle, youbotEuler(3)); % perform the rotation
    
    if (abs(angdiff(new_angle, youbotEuler(3))) < 1 / 180 * pi) && (abs(angdiff(prevOrientation, youbotEuler(3)) < 0.1 / 180 * pi)) 
        rotateRightVel = 0;
        fsm = 'moving';     % ready to move to the next intermediate point of the trajectory
        
        %compute some of the distances used when 'moving'
        prev_position = [youbotPos(1) youbotPos(2)];    % initial position of the robot when it was rotating
        initial_dist = sqrt((trajectory(target,1) - (precision*youbotPos(1)-minMap(1)+1))^2 +...     % inital distance compared to the next destination 
                            (trajectory(target,2) - (precision*youbotPos(2)-minMap(2)+1))^2);        % of the trajectory
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

elseif strcmp (fsm, 'moving')
    
    distance = sqrt((trajectory(target, 1) - resol*youbotPos(1))^2 +...
                    (trajectory(target, 2) - resol*youbotPos(2))^2);    % compute the distance wrt the target
    
    if initial_dist < 2*max_distance         % see if the complete distance is long enough to perform an acceleration/decceleration on max_distance
        half_dist = initial_dist/2;          % if not, acceleration/decceleration on half the distance to run

        % Acceleration.
        forwBackVel = -max_speed/max_distance * (inital_dist - distance);       % negative value of BackVel to go forward

        % Decceleration.
        if distance < half_distance          % when half the distance is crossed, deccelerate
            forwBackVel = -max_speed/max_distance * distance;
        end

    else
        if distance > initial_distance - max_distance
            forwBackVel = -max_speed/max_distance * (inital_dist - distance);   % set an increasing speed, depending on the crossed distance
        elseif distance < initial_distance - max_distance && distance > max_distance
            forwBackVel = -maxSpeed;
        else
            forwBackVel = -max_speed/max_distance * distance;                   % set a decreasing speed, depending on the remaining distance
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
            new_angl = atan(((precision*youbotPos(1)-minMap(1)+1)-trajectory(target,1))/...
                            (trajectory(target,2)-(precision*youbotPos(2)-minMap(2)+1)));   % target has been increased, compute the angle wrt the nex target
            if (trajectory(target,2)-(precision*youbotPos(2)-minMap(2)+1)) > 0           % used to correct the angle as it is computed 
                new_angl = new_angl + pi;                                                     % based on map coordinates, and not robot referential
            end
        end
    end
    
    prevPosition = [youbotPos(1) youbotPos(2)];         % save current position before next computation to see if speed is low enough when stopping the robot
    
end


