path = [   
   -1.8750   -5.3750
   -1.8750   -5.5000
   -1.8750   -5.6250
   -1.8750   -5.7500
   -1.8750   -5.8750
   -1.8750   -6.0000
   -2.0000   -6.1250
   -2.0000   -6.2500
   -2.1250   -6.3750
   -2.2500   -6.5000
   -2.3750   -6.6250
   -2.5000   -6.6250
   -2.6250   -6.6250
   -2.7500   -6.6250
   -2.8750   -6.5000
   -3.0000   -6.3750];


initialOrientation = 3.1401-pi/2;
robotCurrentLocation = [-2.0000   -5.2503];
robotCurrentPose = [robotCurrentLocation initialOrientation];
robotGoal = [ -3.1250   -6.2500];

robotRadius = 0.4;
robot = ExampleHelperRobotSimulator('emptyMap',2);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);
robot.setRobotPose(robotCurrentPose);

plot(path(:,1), path(:,2),'k--d')
xlim([-5 6])
ylim([-7 6])

controller = robotics.PurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.5;
goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation - robotGoal);
controlRate = robotics.Rate(10);
while( distanceToGoal > goalRadius )

    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = step(controller,robot.getRobotPose);
    disp(v);
    disp(omega);
    % Simulate the robot using the controller outputs.
    drive(robot, v, omega);

    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    robotCurrentPose = robot.getRobotPose;

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);

    waitfor(controlRate);

end

