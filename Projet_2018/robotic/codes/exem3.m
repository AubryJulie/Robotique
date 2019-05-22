close all;

% This file requires the following scripts:
%   ptranimate.m
%   distanimate.m
%   rpyanimate.m
% You will find them within the matlab_functions directory of the course's Git.

% Note: the three scripts above are hacks that work for the illustrations below,
% but are not guaranteed to work in all situations.
% Use ptranimate, distanimate, rpyanimate at your own risk.

fps = 20;
frames = 50;
R0 = rotz(-1) * roty(-1);
R1 = rotz(1) * roty(1);
%R0 = rpy2r(2 , pi/2-.5, .2);
%R1 = rpy2r(.5 , pi/2+.5, .1);
%R0 = rpy2r(-3 , 0, -3);
%R1 = rpy2r(3 , pi/2, 3);

rpy0 = tr2rpy(R0); rpy1 = tr2rpy(R1);
rpy = mtraj(@tpoly, rpy0, rpy1, frames);
q0 = Quaternion(R0);
q1 = Quaternion(R1);
if qdot(q0, q1) < 0,
  q1 = qminus(q1);
end
q = interp(q0, q1, [0:frames-1]'/(frames-1));

trplot(R0, 'color', 'r');
hold on;
trplot(R1, 'color', 'g');
tranimate( rpy2tr(rpy), 'fps', fps );
ptranimate( rpy2tr(rpy), 'fps', fps );

plot_sphere([0, 0, 0], 1, [.9 .9 .9]);
lighting gouraud; light; hold on;

ptranimate(q, 'fps', fps )

figure
distanimate( rpy2tr(rpy), 'fps', fps );
distanimate( q, 'fps', fps );

figure
rpyanimate( rpy2tr(rpy), 'fps', fps );
rpyanimate( q, 'fps', fps );
plane = [-pi, pi/2, -pi ; -pi, pi/2, pi ; pi, pi/2, pi ; pi, pi/2, -pi];
patch(plane(:,1), plane(:,2), plane(:,3), 'r');
alpha(0.3)

T0 = transl(0.4, 0.2, 0) * trotx(pi);
T1 = transl(-0.4, -0.2, 0.3) * troty(pi/2)*trotz(-pi/2);
Ts = trinterp(T0, T1, [0:49]/49);
tranimate(Ts)
P = transl(Ts);
plot(P);