function [find, new_case] = trajectory_aux(plan, plan_explored, youbot_trajectory_cel,...
    target_plan_x, target_plan_y)
% plan : 2D map
% plan_explored: binary table containing true if the case as already been
% explored and false otherwise.
% youbot_trajectory_cel: trajectory_cell with the youbot position and
% previous = nan.
% case_plan_x, case_plan_y: the position of the current case in the
% map coordinates.
% target_plan_x, target_plan_y: the target position in the
% map coordinates (must be a free-cell).
% traj_x, traj_y: vector containing the a sequence of case defining the
% trajectory.

[len, width] = size(plan);
% tab: ordered table containing trajectory_cel object.
F = magic(1,len*width);
tab = trajectory_cel(F);
find = false;
i = 1;
j = 2;
tab(i) = youbot_trajectory_cel;
while ~find
    current_case = tab(i);
    d = current_case.distance;
    new_case.d = d + 0.1;%distance in meter
    new_case.previous = current_case;
    x_coord = current_case.x;
    y_coord = current_case.y;
    %for the up, down , left, right case.
    for x = -1:2:1
        for y = -1:2:1
            new_x = x_coord + x;
            new_y = y_coord + y;
            % Check if the new_case is a free_cell in the plan and or if it has already
            % been explored. (free_cell = 1)
            if (plan(new_x, new_y) == 1 && new_x > 0 && new_y > 0 && new_x <= len && new_y <=...
                    width && ~plan_explored(new_x,new_y))
                new_case.x = new_x;
                new_case.y = new_y;
                if target_plan_x == new_x && new_y == target_plan_y
                    find = true;
                    return 
                end
                %add at the end of the tab
                tab(j) = new_case;
                plan_explored(new_x,new_y);
                j = j+1;
            end
        end
    end
    i = i+1;
end
