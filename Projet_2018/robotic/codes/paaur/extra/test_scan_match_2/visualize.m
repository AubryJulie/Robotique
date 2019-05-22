n_pts = 50;


% Scans with pose
% figure(1)
% for j = 1:100
%     sc1 = reshape(scans(j,1,:), size(scans,3), 1);
%     sc2 = reshape(scans(j,2,:), size(scans,3), 1);
%     plot(sc1'+pose(j,1), sc2'+pose(j,2), 'r.', pose(j,1), pose(j,2), 'b*');
%     axis([-8 8 -8 8]);
%     drawnow;
%     pause(0.3);
% end


% % Cheat trans 1 versus Cheat trans 2 (with scores)
% figure(2)
% 
% subplot(3,1,1)
% plot(1:n_pts, cheat_init_transf_v1(:,1), 'bo', 1:n_pts, cheat_init_transf_v2(:,1), 'r*');
% 
% subplot(3,1,2)
% plot(1:n_pts, cheat_init_transf_v1(:,2), 'bo', 1:n_pts, cheat_init_transf_v2(:,2), 'r*');
% 
% subplot(3,1,3)
% scatter(1:n_pts, scores);
% 
% 
% % Cheat trans 2 versus init trans (with scores)
% figure(3)
% 
% subplot(3,1,1)
% plot(1:n_pts, cheat_init_transf_v1(:,1), 'bo', 1:n_pts, init_transf(:,1), 'r*');
% 
% subplot(3,1,2)
% plot(1:n_pts, cheat_init_transf_v1(:,2), 'bo', 1:n_pts, init_transf(:,2), 'r*');
% 
% subplot(3,1,3)
% scatter(1:n_pts, scores);
% 
% 
% % Real trans versus estimated trans (with scores) (!!!)
% for j = 1:n_pts
%     if scores(j) < 100
%         estim_transf(j,:) = cheat_init_transf_v1(j,:);
%     end
% end
figure(4)

subplot(3,1,1)
plot(1:n_pts, cheat_init_transf_v1(1:n_pts,1), 'bo', 1:n_pts, 0.1*estim_transf(1:n_pts,1), 'r*');

subplot(3,1,2)
plot(1:n_pts, cheat_init_transf_v1(1:n_pts,2), 'bo', 1:n_pts, 0.1*estim_transf(1:n_pts,2), 'r*');

subplot(3,1,3)
scatter(1:n_pts, scores(1:n_pts));
% 
% 
% Path versus estimated path
figure(5)
plot(pose(1:n_pts,1), pose(1:n_pts,2) , 'ro', est_pose(1:n_pts,1), est_pose(1:n_pts,2) , 'b*');
axis([-8 8 -8 8]);
