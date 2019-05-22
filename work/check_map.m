function [complete, next_pos_x, next_pos_y] = check_map(plan, youbot_pos_plan_x, youbot_pos_plan_y)
% Verify if a map is complete.
% plan : 2D map
% youbot_pos_plan_x, youbot_pos_plan_y: the position of the youbot in the
% map coordinates.
% complete : boolean
% next_pos_x, next_pos_y : the next position to explore if the map is not
% complete.

% hokuyo range = 5m -> first check in a 6m square around the robot.
[len, width] = size(plan);
next_pos_x = 0;
next_pos_y = 0;
complete = true;
find = 0;
x_min = youbot_pos_plan_x-60;
y_min = youbot_pos_plan_y-60;
x_max = youbot_pos_plan_x+60;
y_max = youbot_pos_plan_y+60;

% To not exceed the plan coordinate
if x_min <1
    x_min = 1;
end
if y_min < 1
    y_min = 1;
end
if x_max > len
    x_max = len;
end
if y_max > width
    y_max = width;
end
% Check in a square of 6m around the youbot if a case is unexplored.
for i = x_min:x_max
    for j = y_min:y_max
        current = plan(i,j);
        if i ~= len
            down = plan(i+1,j);
        else
            down = current;
        end
        if j ~= width
            right = plan(i,j+1);
        else
            right = current;
        end
        % Check if there is a free_cell (= 1) close to an
        % unexplored_cell (= 0).
        if current + right == 1 || current + down == 1
            complete = false;
            if current == 1
                next_pos_x = i;
                next_pos_y = j;
                % Check if the free_cell neighbors are surrounded by
                % free_cells only. Use to find a reachable target.
                for a = -1:1
                    for b = -1:1
                        neighbor_pos_x = next_pos_x + a;
                        neighbor_pos_y = next_pos_y + b;
                        free = true;
                        for c = -1:1
                            for d = -1:1
                                free = free && (plan(neighbor_pos_x+c, neighbor_pos_y+d) == 1);
                            end
                        end
                        if free
                            next_pos_x = neighbor_pos_x;
                            next_pos_y = neighbor_pos_y;
                            find = 1;
                            return
                        end
                    end
                end
            elseif right == 1
                next_pos_x = i;
                next_pos_y = j+1;
                % Check if the free_cell neighbors are surrounded by
                % free_cells only. Use to find a reachable target.
                for a = -1:1
                    for b = -1:1
                        neighbor_pos_x = next_pos_x + a;
                        neighbor_pos_y = next_pos_y + b;
                        free = true;
                        for c = -1:1
                            for d = -1:1
                                free = free && (plan(neighbor_pos_x+c, neighbor_pos_y+d) == 1);
                            end
                        end
                        if free
                            next_pos_x = neighbor_pos_x;
                            next_pos_y = neighbor_pos_y;
                            find = 1;
                            return
                        end
                    end
                end
            elseif down == 1
                next_pos_x = i+1;
                next_pos_y = j;
                % Check if the free_cell neighbors are surrounded by
                % free_cells only. Use to find a reachable target.
                for a = -1:1
                    for b = -1:1
                        neighbor_pos_x = next_pos_x + a;
                        neighbor_pos_y = next_pos_y + b;
                        free = true;
                        for c = -1:1
                            for d = -1:1
                                free = free && (plan(neighbor_pos_x+c, neighbor_pos_y+d) == 1);
                            end
                        end
                        if free
                            next_pos_x = neighbor_pos_x;
                            next_pos_y = neighbor_pos_y;
                            find = 1;
                            return
                        end
                    end
                end
            end
        end
    end
end
% If we don't have find in the small square check all the plan
if ~find
    for i = 1:len
        for j = 1:width
            current = plan(i,j);
            if i ~= len
                down = plan(i+1,j);
            else
                down = current;
            end
            if j ~= width
                right = plan(i,j+1);
            else
                right = current;
            end
            % Check if there is a free_cell (= 1) close to an
            % unexplored_cell (= 0).
            if current + right == 1 || current + down == 1
                complete = false;
                if current == 1
                    next_pos_x = i;
                    next_pos_y = j;
                    % Check if the free_cell neighbors are surrounded by
                    % free_cells only. Use to find a reachable target.
                    for a = -1:1
                        for b = -1:1
                            neighbor_pos_x = next_pos_x + a;
                            neighbor_pos_y = next_pos_y + b;
                            free = true;
                            for c = -1:1
                                for d = -1:1
                                    free = free && (plan(neighbor_pos_x+c, neighbor_pos_y+d) == 1);
                                end
                            end
                            if free
                                next_pos_x = neighbor_pos_x;
                                next_pos_y = neighbor_pos_y;
                                return
                            end
                        end
                    end
                elseif right == 1
                    next_pos_x = i;
                    next_pos_y = j+1;
                    % Check if the free_cell neighbors are surrounded by
                    % free_cells only. Use to find a reachable target.
                    for a = -1:1
                        for b = -1:1
                            neighbor_pos_x = next_pos_x + a;
                            neighbor_pos_y = next_pos_y + b;
                            free = true;
                            for c = -1:1
                                for d = -1:1
                                    free = free && (plan(neighbor_pos_x+c, neighbor_pos_y+d) == 1);
                                end
                            end
                            if free
                                next_pos_x = neighbor_pos_x;
                                next_pos_y = neighbor_pos_y;
                                return
                            end
                        end
                    end
                elseif down == 1
                    next_pos_x = i+1;
                    next_pos_y = j;
                    % Check if the free_cell neighbors are surrounded by
                    % free_cells only. Use to find a reachable target.
                    for a = -1:1
                        for b = -1:1
                            neighbor_pos_x = next_pos_x + a;
                            neighbor_pos_y = next_pos_y + b;
                            free = true;
                            for c = -1:1
                                for d = -1:1
                                    free = free && (plan(neighbor_pos_x+c, neighbor_pos_y+d) == 1);
                                end
                            end
                            if free
                                next_pos_x = neighbor_pos_x;
                                next_pos_y = neighbor_pos_y;
                                return
                            end
                        end
                    end
                end
            end
        end
    end
end