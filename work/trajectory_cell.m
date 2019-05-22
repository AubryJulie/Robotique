classdef trajectory_cell
    properties
	% Position in map coordinates
        x;
        y;
	% Previous trajectory_cell
        previous;
    end
    
    methods
		% Create an array of trajectory_cell
        function obj = trajectory_cell(F)
            if nargin ~= 0
                m = size(F,1);
                n = size(F,2);
                obj(m,n) = obj;
                for i = 1:m
                    for j = 1:n
                        obj(i,j).x = F(i,j);
                        obj(i,j).y = F(i,j);
                    end
                end
            end
        end
		% return an array of trajectory_cell defining a trajectory.
        function traj = traject(obj)
            traj = [];
            while isobject(obj)
                traj = [obj,traj];
                obj = obj.previous;
            end
        end
    end
end