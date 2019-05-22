function [objects] = detecObjects(pts_world)
    colors = [0 0 1;0 1 0; 1 0 0;0 0 0;1 1 0];

    score = Inf;
    for i = 1 : 100
        [idx,C,sumd] = kmeans([pts_world(1,:);pts_world(2,:)],5);
        if(sumd < score)
            score = sumd;
            C_ref = C;
            idx_ref = idx;
        end
    end
    ROIs = zeros(3,2,5);
    ptsROIs = cell(5,1);
    minH = Inf;
    for i = 1 : 5
        if(max(pts_world(3,idx_ref==i)) < minH)
            minH = max(pts_world(3,idx_ref==i));
        end
        pts_1 = pts_world(1:3,pts(3,:) < minH);
    end
    ptCloud = pointCloud(pts_1');
    ptCloud = pcdenoise(ptCloud);

    for i = 1 : 5
        ROIs(:,:,i) = [ C_ref(1,i) - 0.05, C_ref(1,i) + 0.05; C_ref(2,i) - 0.05, C_ref(2,i) + 0.05 ; 0.18, minH ];
        indices = findPointsInROI(ptCloud,ROIs(:,:,i));
        tmp = ptCloud.Location;
        ptsROIs{i} = indices;
        subplot(121);
        pcshow(tmp(indices,:),colors(i,:));
        hold on;
    end

    %detect cube
    objects(5) = struct('center',[],'shape',[],'color',[]); 
    maxDistance = 0.00005;
    for i = 1 : 5
        [model,inlierIndices,outlierIndices,meanError] = pcfitplane(ptCloud,maxDistance,...
                              'SampleIndices',ptsROIs{i},'confidence',99.99999,'MaxNumTrials',3000);
        size(inlierIndices,1)
         height = mean(pts_world(3,idx_ref==i));
        if(~isempty(model) && size(inlierIndices,1) > 100)
            objects(i).shape = 'box';
            plot3(C_ref(1,i),C_ref(2,i),height,'rx','MarkerSize',15,'LineWidth',3);
        else
            objects(i).shape = 'cylinder';
            plot3(C_ref(1,i),C_ref(2,i),height,'kx','MarkerSize',15,'LineWidth',3);
        end
        objects(i).center = [C_ref(1,i) C_ref(2,i) height];

    end
end