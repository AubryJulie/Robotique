close all;
clear;
clc;


mapname = '9ec55ebd-9e1f-4a30-9032-f68f6d4f89d1.mat'; % 160 resol
% mapname = 'a76b3d1a-885f-4eed-9bfa-09972b1a1937.mat'; % 80 resol


load(mapname)
m = mat;
m(m<0.5)=0;
m(m>=0.5)=1;

BW2 = bwmorph(m, 'close');
figure
imshow(BW2);
[centers,radii,c] = imfindcircles(BW2,[3 11],'ObjectPolarity','bright', 'Method', 'TwoStage', 'Sensitivity', 0.8, 'EdgeThreshold', 0.55);
viscircles(centers, radii,'EdgeColor','b');

map34 = robotics.OccupancyGrid(BW2);
pose=[60 35 -pi/2];
angles = linspace(-4*pi/6, 4*pi/6, 250);
maxrange = 50;
pts = rayIntersection(map34,pose,angles,maxrange);
pts = pts(~isnan(pts(:,1)),:);
pts = world2grid(map34, pts);

hold on;
plot (pts(:,2), pts(:,1), 'ro');
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
unique(selected, 'rows');