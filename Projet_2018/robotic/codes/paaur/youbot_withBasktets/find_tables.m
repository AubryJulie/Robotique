function [tables] = find_tables(mat, startIJ, startAngle)

    m = mat;
    m(m<0.5)=0;
    m(m>=0.5)=1;

    BW2 = bwmorph(m, 'close');

    [centers,radii,~] = imfindcircles(BW2,[3 11],'ObjectPolarity','bright', 'Method', 'TwoStage', 'Sensitivity', 0.8, 'EdgeThreshold', 0.55);

    map34 = robotics.OccupancyGrid(BW2);
    pose=[startIJ startAngle-pi/2];
    xy=grid2world(map34,pose(1:2));
    pose = [xy pose(3)];
    angles = linspace(-4*pi/6, 4*pi/6, 250);
    maxrange = 50;
    pts = rayIntersection(map34,pose,angles,maxrange);
    pts = pts(~isnan(pts(:,1)),:);
    pts = world2grid(map34, pts);
 
    selected = [];
    for i=1:size(pts,1)
        if (~isnan(pts(i,1)))
            for j = 1:size(centers,1)
                dist = norm(pts(i,:)-[centers(j,2) centers(j,1)]);
                if dist < 10
                    selected = [selected; centers(j,:)];
                end
            end
        end
    end
    tables = unique(selected, 'rows');
end
