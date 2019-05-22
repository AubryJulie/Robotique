V = diag([0.02, 0.5*pi/180].^2);
veh = Vehicle(V)
odo = veh.step(1, 0.3)
veh.add_driver( RandomPath(10) )
veh.run()


veh.Fx( [0,0,0], [0.5, 0.1] )
P0 = diag([0.005, 0.005, 0.001].^2);
ekf = EKF(veh, V, P0);
ekf.run(1000);
veh.plot_xy()
hold on
ekf.plot_xy('r')
P700 = ekf.history(700).P
ekf.plot_ellipse([], 'g')
ekf.plot_P();



map = Map(20);
veh = Vehicle([]); % error free vehicle veh.add_driver( RandomPath(map.dim) );
W = diag([0.1, 1*pi/180].^2);
sensor = RangeBearingSensor(veh, map, W);
ekf = EKF(veh, [], [], sensor, W, []);
ekf.run(1000);

W = diag([0.1, 1*pi/180].^2);
V = diag([0.02, 0.5*pi/180].^2);
P0 = diag([.01, .01, 0.005].^2);
map = Map(20);
veh = Vehicle(V);
veh.add_driver( RandomPath(map.dim) );
sensor = RangeBearingSensor(veh, map, W); ekf = EKF(veh, V, P0, sensor, W, []);
ekf.run(1000);
map.plot();
ekf.plot_map(5, 'g');
ekf.plot_xy('r');
veh.plot_xy('b');





map = Map(20);

W = diag([0.1, 1*pi/180].^2);
veh = Vehicle(W);
veh.add_driver( RandomPath(10) );

V = diag([0.005, 0.5*pi/180].^2);
sensor = RangeBearingSensor(veh, map, V);

Q = diag([0.1, 0.1, 1*pi/180]).^2;

L = diag([0.1 0.1]);

pf = ParticleFilter(veh, sensor, Q, L, 1000);


pf.run(1000);



map.plot();

veh.plot_xy('b');

pf.plot_xy('r');

plot(pf.std(1:100,:))

pf.plot_pdf()