function [complete, next_pos_x, next_pos_y] = check_map(plan)
% Verify if a map is complete.
% plan : 2D map
% complete : boolean
% next_pos_x, next_pos_y : the next position to explore if the map is not
% complete.

complete = 1;
[len, width] = size(plan);
next_pos_x = 0;
next_pos_y = 0;
for i = 1:len
    for j = 1:width
        current = plan(i,j);
        if i ~= len
            down = plan(i+1,j);
        end
        if j ~= width
            right = plan(i,j+1);
        end
        if current + right == 1 || current + down == 1
            complete = 0;
            if current == 1
              next_pos_x = i;
              next_pos_y = j;
            elseif right == 1
                next_pos_x = i;
                next_pos_y = j+1;
            elseif down == 1
                next_pos_x = i+1;
                next_pos_y = j;                
            end
            break;
        end
    end
end