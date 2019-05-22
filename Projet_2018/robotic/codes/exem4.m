sl_lanechange

sl_drivepoint
x0 = [8 5 pi/2];
xg = [5 5];
r = sim('sl_drivepoint');
q = r.find('yout');
plot(q(:,1), q(:,2));

sl_driveline
L = [1 -2 4];
x0 = [8 5 pi/2];
r = sim('sl_driveline');
t = r.find('tout'); y = r.find('yout');
figure, hold on, plot(y(:,1), y(:,2));
plot_homline(L', 'r');

sl_pursuit
r = sim('sl_pursuit')

sl_braitenberg
sim('sl_braitenberg');

load map1
bug = Bug2(map);
bug.goal = [50; 35];
bug.path();

% Distance transform:

goal = [50; 30];
start = [20; 10];
load map1;
dx = DXform(map);
dx.plan(goal, 0.1);
dx.path(start);

close all
goal = [50; 30];
start = [20; 10];
load map1;
dx = DXform(map);
dx.plan(goal);
p = dx.path(start);
dx.plot(p)
figure
dx.plot3d(p)

% D Star

goal = [50; 30];
start = [20; 10];
load map1;
ds = Dstar(map);
c = ds.costmap_get();
ds.plan(goal);
ds.path(start);

% Voronoi

free = 1 - map;
free(1,:) = 0; free(100,:) = 0;
free(:,1) = 0; free(:,100) = 0;
skeleton = ithin(free, 1);

% PRM

prm = PRM(map)
prm.plan()
prm.plot()
prm.path(start, goal);
prm = PRM(map, 'inflate', 3);

% RTT

rrt = RRT()
rrt.plan();
rrt.plot();