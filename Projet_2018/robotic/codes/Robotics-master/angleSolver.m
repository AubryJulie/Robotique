close all;
% Variables of the youbot
l0 = 0.147+0.14; % [m] height before the first horizontal hinge
l1 = 0.155; %[m] length of the first arm starting from the bottom
l2 = 0.135; %[m] length of the second arm starting from the bottom
l3 = 0.218; %[m] length of the third arm starting from the bottom (up to the tip of the gripper)

Xc = 0.3294-l3+0.1; %[m]

yObject = 0.2101-l0+0.05; %[m] height of the horizontal line passing trough the point to grab

ThetaA = zeros(1,length(Xc));
ThetaB = zeros(1,length(Xc));

% We use matlab's symbolic to solve the nonlinear problem

for i=1:length(Xc)
  i
  syms thetaA;
  syms thetaB;

  equation1 = Xc(i) == l1*sin(thetaA)+l2*sin(thetaB+thetaA);
  equation2 = yObject == l1*cos(thetaA)-l2*cos(pi-(thetaB+thetaA));

  solution = vpasolve([equation1 equation2], [thetaA thetaB], [-1.5707963705063, 1.308996796608; -2.2863812446594, 2.2863812446594]);

  % TODO what if no solution is found!

  ThetaA(i) = double(solution.thetaA)*180/pi;
  ThetaB(i) = double(solution.thetaB)*180/pi;
end

ThetaC = 90-(ThetaA+ThetaB);

figure('Name','Angles evolution');
plot(Xc,ThetaA);
hold on;
plot(Xc,ThetaB);
plot(Xc,ThetaC);
hold off;

figure('Name','Animation');
for i=1:length(Xc)

  PtA = [0 0];
  PtB = [l1*sind(ThetaA(i)) l1*cosd(ThetaA(i))];
  PtC = [l1*sind(ThetaA(i))+l2*sind(ThetaB(i)+ThetaA(i)) l1*cosd(ThetaA(i))-l2*cosd(180-(ThetaB(i)+ThetaA(i)))];
  PtD = [l1*sind(ThetaA(i))+l2*sind(ThetaB(i)+ThetaA(i))+l3*sind(ThetaC(i)+ThetaB(i)+ThetaA(i)) ...
   l1*cosd(ThetaA(i))-l2*cosd(180-(ThetaB(i)+ThetaA(i)))+l3*cosd(180-(ThetaC(i)+ThetaB(i)+ThetaA(i)))];

  clf;
  hold on;
  plot([PtA(1) PtB(1) PtC(1) PtD(1)],[PtA(2) PtB(2) PtC(2) PtD(2)],'-o');
  axis equal;
  %axis([0 0.5 0 0.5]);
  pause(0.3);
end
