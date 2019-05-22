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
    if x_coord == 0
        plan_explored(target_plan_x,target_plan_y) = 2;
        figure(2)
        contour(plan_explored);
        %         for p = 1:j
        %             tab(p)
        %         end
        target_plan_x
        target_plan_y
        far
        error('error in trajectory for y');
    end
    %for the up, down , left, right case.
    for x = -1:1:1
        for y = -1:1:1
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
                    % Check if the target is close
                    close_target = false;
                    for k =-2:2
                        for  p = -2:2
                            close_target = close_target ||((new_x+k == target_plan_x) && (target_plan_y == new_y+p));
                        end
                    end
                    if close_target
                        %if ((target_plan_x == new_x && new_y == target_plan_y))% || ...
                        %(target_plan_x-1 == new_x && new_y == target_plan_y) || ...
                        %(target_plan_x+1 == new_x && new_y == target_plan_y) || ...
                        %(target_plan_x == new_x && new_y == target_plan_y-1) || ...
                        %(target_plan_x == new_x && new_y == target_plan_y+1))
                        %                         plan_explored(target_plan_x,target_plan_y) = 2;
                        %                         figure(2)
                        %                         contour(plan_explored);
                        %                         target_plan_x
                        %                         target_plan_y
                        %                         target_plan_y
                        %                         new_y
                        find = true;
                        break;
                    end
                    % Retrieve from the trajectory cases too close from
                    % obstacles
                    far = true;
                    for k =-2:2
                        for  p = -2:2
                            if ~((k == 0) && (p == 0))
                                far = far && (plan(new_x+k,new_y+p) == 1);
                            end
                        end
                    end
                    %                     if ~far
                    %                         plan_explored(new_x,new_y) = 3;
                    %                         for k =-1:1
                    %                             for  p = -1:1
                    %                                 if ~((k == 0) && (p == 0))
                    %                                     plan(new_x+k,new_y+p)
                    %                                 end
                    %                             end
                    %                         end
                    %
                    %                         figure(3)
                    %                         contour(plan)
                    %                         new_x
                    %                         new_y
                    %                     end
                    if far
                        %                     if ~((plan(new_x-1, new_y) == 2) || (plan(new_x+1, new_y) == 2) || ...
                        %                             (plan(new_x, new_y-1) == 2) || (plan(new_x, new_y+1) == 2) ||...
                        %                             (plan(new_x-2, new_y) == 2) || (plan(new_x+2, new_y) == 2) || ...
                        %                             (plan(new_x, new_y-2) == 2) || (plan(new_x, new_y+2) == 2) || ...
                        %                             (plan(new_x-1, new_y-1) == 2) || (plan(new_x+1, new_y+1) == 2) || ...
                        %                             (plan(new_x-1, new_y+1) == 2) || (plan(new_x+1, new_y-1) == 2) ||...
                        %                             (plan(new_x-1, new_y) == 2) || (plan(new_x+1, new_y) == 2) || ...
                        %                             (plan(new_x, new_y-1) == 2) || (plan(new_x, new_y+1) == 2) ||...)
                        %add at the end of the tab
                        tab(j) = new_case;
                        plan_explored(new_x,new_y) = 1;
                        %                         figure(2)
                        %                         contour(plan_explored);
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