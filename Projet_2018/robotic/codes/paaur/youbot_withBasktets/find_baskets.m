function [baskets] = find_baskets(mat)

    baskets = []; % center(ij) accesspoint(ij)
    m = mat;
    m(m<0.5)=0;
    m(m>=0.5)=1;

    BW2 = bwmorph(m, 'close');
    [a,~,~] = imfindcircles(BW2,[3 11],'ObjectPolarity','bright', 'Method', 'TwoStage', 'Sensitivity', 0.8, 'EdgeThreshold', 0.55);

    hormat = ones(1,15);
    vermat = ones(15,1);
    horvals = conv2(double(BW2), hormat, 'same');
    vervals = conv2(double(BW2), vermat, 'same');

    pts = floor(a);
    meta_filtered = [];
    sum = horvals + vervals;
    corners = detectHarrisFeatures(BW2); 
    corners = floor(corners.Location);
    for i = 1:size(pts,1)
        ra = pts(i,2)-5;
        if (ra < 1)
            ra = 1;
        end
        rb = pts(i,2)+5;
        if (rb > 160)
            rb = 160;
        end
        ca = pts(i,1)-5;
        if (ca < 1)
            ca = 1;
        end
        cb = pts(i,1)+5;
        if (cb > 160)
            cb = 160;
        end

        v = sum(ra:rb, ca:cb);
        if ~isempty(find(v>=25))
            cnt_corners = zeros(4,1);
            tmp_corners = size(find(corners(:,2) >= ra & corners(:,2) <= pts(i,2) & corners(:,1) >= ca & corners(:,1) <= pts(i,1)),1);
            cnt_corners(1) = tmp_corners;
            tmp_corners = size(find(corners(:,2) >= ra & corners(:,2) <= pts(i,2) & corners(:,1) >= pts(i,1) & corners(:,1) <= cb),1);
            cnt_corners(2) = tmp_corners;
            tmp_corners = size(find(corners(:,2) >= pts(i,2) & corners(:,2) <= rb & corners(:,1) >= ca & corners(:,1) <= pts(i,1)),1);
            cnt_corners(3) = tmp_corners;
            tmp_corners = size(find(corners(:,2) >= pts(i,2) & corners(:,2) <= rb & corners(:,1) >= pts(i,1) & corners(:,1) <= cb),1);
            cnt_corners(4) = tmp_corners;

            [~,priority] = sort(cnt_corners, 'descend');
            meta_filtered = [meta_filtered; pts(i,:) priority'];
        end
    end


    % ALGO DE DECISION

    inflateFactor = 5;
    map = robotics.OccupancyGrid(BW2);
    mat = occupancyMatrix(map,'ternary');
    temp = mat;
    mat(mat == 1) = 1;
    mat(mat == -1) = 1;
    disp = mat;

    inflate(map, inflateFactor);
    mat = occupancyMatrix(map,'ternary');
    mat(mat == 1) = 1;
    mat(mat == -1) = 1;
    m1 = robotics.OccupancyGrid(mat);
    mat = occupancyMatrix(map,'ternary');
    mat(mat == -1) = 1;
    m2 = robotics.OccupancyGrid(mat);
    inflate(m1, .15);

    mat1 = occupancyMatrix(m1,'ternary');
    mat2 = occupancyMatrix(m2,'ternary');
    [r,c] = find(mat1 == 1 & mat2 == 0); % frontiers of unknown points
    pts = [r,c];

    n_obs = size(meta_filtered, 1);
    obs = zeros(n_obs,2);
    for i = 1:n_obs
        priority = meta_filtered(i,3:6);
        center = meta_filtered(i,1:2);
        for j = 1:4
            switch priority(j)
                case 1
                    [a1,~] = find(pts(:,1) < center(2));
                    [a2,~] = find(pts(:,2) < center(1));
                    a = intersect(a1,a2);
                case 2
                    [a1,~] = find(pts(:,1) < center(2));
                    [a2,~] = find(pts(:,2) > center(1));
                    a = intersect(a1,a2);
                case 3
                    [a1,~] = find(pts(:,1) > center(2));
                    [a2,~] = find(pts(:,2) < center(1));
                    a = intersect(a1,a2);
                case 4
                    [a1,~] = find(pts(:,1) > center(2));
                    [a2,~] = find(pts(:,2) > center(1));
                    a = intersect(a1,a2);
            end
            mypts = pts(a,:);
            if isempty(mypts)
                continue
            end
            n_pts = size(mypts,1);
            min_dist = Inf;
            for k = 1:n_pts
                dist = norm(mypts(k,:)-[center(2) center(1)]);
                if (dist < min_dist)
                    obs(i,:) = mypts(k,:);
                    min_dist = dist;
                end
            end
            if min_dist ~= Inf
                break;
            end
       end
    end
    size(meta_filtered)
    size(obs)
    baskets = [meta_filtered(:,2) meta_filtered(:,1) obs(:,1:2)];
end





