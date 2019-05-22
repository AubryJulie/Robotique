figure;
subplot(121);
pcshow(ptCloud);
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
title('Detect a Cylinder in a Point Cloud');
tic;
n = 0;
center = [0 0];
rad = 0;
for i = 1 : 5
    maxDistance = 0.00005;
    referenceVector = [0,0,1];
    [model,inlierIndices] = pcfitcylinder(ptCloud,maxDistance,referenceVector,'confidence',0.99999);
    if(model.Radius < 0.42 && model.Radius > 0.38)
        center = (center + model.Center(1:2));
        rad = (rad + model.Radius);
        n = n + 1;
        hold on;
        plot(model);
    end
end
center = center/n;
rad = center/n;
toc
hold on;
center