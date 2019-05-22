% ------ Loading data ------ %
load('data/result');
angls = angls_;
rngs = rngs_;
pose = pose_;
odom = odometry_;
n_angles = size(angls,2);
n_pts = size(angls,1);

est_pose = zeros(n_pts, 3); 
cheat_init_transf_v1 = zeros(n_pts, 3); % from diff (on real poses)
cheat_init_transf_v2 = zeros(n_pts, 3); % from function matlab (on real poses)
init_transf = zeros(n_pts, 3); % (on estimated poses)
estim_transf = zeros(n_pts, 3);
scores = zeros(n_pts, 1);
scans = zeros(n_pts, 2, n_angles);

% ------ Check dimensions ------ %
disp('-- Dimensions --');
disp('size angls: ');
size(angls)
disp('size rngs: ');
size(rngs)
disp('size pose: ');
size(pose)
disp('size odometry: ');
size(odom)


% ------ Compute ------ %
est_pose(1,:) = pose(1,:);
for j = 1:50
        angls(j,:) = pose(j,3) + angls(j,:);
        %angls(j,:) = -pi+angls(j,:);
        
        %[scans(j,1,:) scans(j,2,:)] = pol2cart(pi+pi/8+angls(j,:), rngs(j,:));
        [scans(j,1,:) scans(j,2,:)] = pol2cart(angls(j,:), rngs(j,:));
        
        cheat_init_transf_v1(j,:) = pose(j+1,:)-pose(j,:);
        cheat_init_transf_v2(j,:) = pose(j+1,:)-pose(j,:); % to modify
        init_transf(j,:) = pose(j+1,:)-pose(j,:); % to modify
        
        [transform, stats] = matchScans(rngs(j+1,:), angls(j+1,:), rngs(j,:), angls(j,:), ...
        'SolverAlgorithm', 'fminunc', 'MaxIterations', 500);
    
        estim_transf(j,:) = transform;
        scores(j) = stats.Score;
        
        est_pose(j+1,:) = exampleHelperComposeTransform(est_pose(j,:), [0.25*transform(1) 0.25*transform(2) transform(3)]);
end


    

