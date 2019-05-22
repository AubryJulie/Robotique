function [find, traj] = trajectory(plan, youbot_pos_plan_x, ...
    youbot_pos_plan_y, target_plan_x, target_plan_y)
% plan : 2D map
% youbot_pos_plan_x, youbot_pos_plan_y: the position of the youbot in the
% map coordinates.
% target_plan_x, target_plan_y: the target position in the
% map coordinates (must be a free-cell).
% find: return true if a trajectory exist.
% traj: vector containing the trajectory.

% youbot_trajectory_cell: trajectory_cell with the youbot position and
% previous = nan.
youbot_trajectory_cell = trajectory_cell;
youbot_trajectory_cell.x = youbot_pos_plan_x;
youbot_trajectory_cell.y = youbot_pos_plan_y;
youbot_trajectory_cell.previous = nan;
[len, width] = size(plan);
% plan_explored: matrix containing true if the case as already been
% explored and false otherwise.
plan_explored = zeros(len,width);
% tab: ordered array containing trajectory_cell object.
F = zeros(1,len*width);
tab = trajectory_cell(F);
find = false;
i = 1;
j = 2;
tab(i) = youbot_trajectory_cell;

while ~find
    current_case = tab(i);
    x_coord = current_case.x;
    y_coord = current_case.y;
    %  Debugging of the function when no trajectory is find
    if x_coord == 0
        plan_explored(target_plan_x,target_plan_y) = 2;
        figure(2)
        contour(plan_explored);
        %         for p = 1:j
        %             tab(p)
        %         end
        for k =-4:4
            for  p = -4:4
                plan_explored(target_plan_x+k,target_plan_y+p)
            end
        end
        target_plan_x
        target_plan_y
        far
        error('error in trajectory for y');
    end
    
    %for the up, down, left, right cases (to speed up we don't use the direct neighbor).
    for x = -2:2:2
        for y = -2:2:2
            if (x == 0 || y == 0) && ~(x == 0 && y == 0)
                new_x = x_coord + x;
                new_y = y_coord + y;
                % Check if the new_case is a free_cell in the plan and or if it has already
                % been explored. (free_cell = 1)
                if (new_x > 1 && new_y > 1 && new_x < len && new_y < width ...
                        && (plan(new_x, new_y) == 1) && ~plan_explored(new_x,new_y))
                    new_case = trajectory_cell;
                    new_case.previous = current_case;
                    new_case.x = new_x;
                    new_case.y = new_y;
                    % Remove from the trajectory cases too close from
                    % obstacles
                    far = true;
                    for k =-3:3
                        for  p = -3:3
                            far = far && (plan(new_x+k,new_y+p) == 1);
                        end
                    end
                    if far
                        % Check if the target is close. If it's the case
                        % return the trajectory.
                        close_target = false;
                        for k =-4:4
                            for  p = -4:4
                                close_target = close_target ||((new_x+k == target_plan_x) && (target_plan_y == new_y+p));
                            end
                        end
                        if close_target
                            find = true;
                            break;
                        end
                        %add at the end of the tab
                        tab(j) = new_case;
                        plan_explored(new_x,new_y) = 1;
                        j = j+1;
                    end
                end
            end
        end
        if find
            break;
        end
    end
    % No more trajectory_cell to explore in tab.
    if i==j
        break;
    end
    i = i+1;
end
if find
    traj = new_case.traject();
    plan_explored(target_plan_x,target_plan_y) = 2;
    figure(2)
    contour(plan_explored);
end