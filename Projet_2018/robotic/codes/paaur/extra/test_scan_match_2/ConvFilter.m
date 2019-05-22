classdef ConvFilter
    
    properties (Access = private)
        mat % Matrix representing map (free = 0, obstacle = 0, unknown = 1)
        N % Dimension of filter
        R % Result
    end % end private properties
    
    methods (Access = public)
        %% Constructor
        function obj = ConvFilter(mat, N)
            mat(mat == 1) = 0; % Obstacle cells are seen as known
            mat(mat == -1) = 1; % Unknown cells
            obj.mat = mat;
            obj.N = N;
            obj = obj.conv_filter(); % Apply convolution filter
        end
        
        %% Update Matrix 
        % (used when map has changed)
        function obj = update(obj, mat)
            mat(mat == 1) = 0; % Obstacle cells are seen as known
            mat(mat == -1) = 1; % Unknown cells
            obj.mat = mat;
            obj = obj.conv_filter(); % Apply convolution filter
        end
        
        %% Get the density values for set of points 
        function res = get_cost(obj, pts)
            res = ones(size(pts,1),1);
            for i = 1:size(pts,1)
                res(i) = obj.R(pts(i,1),pts(i,2));
            end
        end
        
        %% Get Matrix
        function res = get_matrix(obj)
            res = obj.R;
        end
    end % end public methods
    
    methods (Access = private)
        %% Apply filter to measure sort of the density of unknown points in a certain region
        function obj = conv_filter(obj)
            A = ones(obj.N); % Matrix used for convolution
            obj.R = conv2(obj.mat, A, 'same'); % Convolution
            obj.R = obj.R / max(obj.R(:)); % Rescale
            obj.R  = ones(size(obj.R)) - obj.R; % Cost is higher for known regions
        end
    end % end private methods
end % end class
        