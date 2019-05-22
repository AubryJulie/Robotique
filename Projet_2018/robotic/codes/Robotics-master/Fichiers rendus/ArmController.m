classdef ArmController <handle

%===============================================================================

properties (Access =private)

% Variables of the youbot
l0 = 0.147+0.14; % [m] height before the first horizontal hinge
l1 = 0.155; %[m] length of the first arm starting from the bottom
l2 = 0.135; %[m] length of the second arm starting from the bottom
l3 = 0.2175-0.08; %[m] length of the third arm starting from the bottom (up to the tip of the gripper)

% Variables of the object to grab
xPointToGrab; % [m] position along x of the point to grab
yPointToGrab; % [m] height of the point to grab

% Joints position
currentJointsPositions = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
initialJointsPositions = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
transportPosition = [0, 35.5 * pi / 180, 55 * pi / 180, 72.68 * pi / 180, 0]; % hold the object against the youbot's surface


% Increment for the lift function
incr = 0.1;

% Variables for the pickup function
armState = 'Rotation';
counter = 0;
liftBoolean = true;
reachBoolean = false;
gripper = false;
initialRadius = 1.5;
isDone = false;
distTipObject;

% Variables for the drop function
tableToDrop; % Table where the object will be dropped
angularPositionToDrop; % Angle of the table where the object will be dropped
dropState;
minimumRadius = 0.8; %[m] The closest the youbot can be from a table
positioningBoolean = true;
dropJoints = [-pi/2, pi/3, pi/6, 0, 0];
gripperRelease = false;


end % End of private properties

%===============================================================================

methods

% Constructor
function obj = ArmController()

end % End of constructor

function [jointsPositions, gripper, isDone] = objectPickup(obj, holonomicController, forwBackVel, leftRightVel, rotVel)
  % First rotate the whole arm
  if strcmp(obj.armState,'Rotation')
    obj.armState = 'Positioning';
    initialRotation(obj);
    obj.counter = 0;

  % Then wait a bit before getting the arm in position the take the object
  elseif strcmp(obj.armState,'Positioning')
    obj.counter = obj.counter+1;
    if obj.counter> 200
      positioning(obj);
      obj.armState = 'Reach';
      obj.counter = 0;
    end

  % Wait a bit before moving the youbot toward the object
  elseif strcmp(obj.armState,'Reach')
    obj.counter = obj.counter+1;
    if obj.counter > 200 && ~obj.reachBoolean
      setNewRadiusNearTable(holonomicController,obj.initialRadius-obj.distTipObject);
      obj.counter = 0;
      obj.reachBoolean = true;
    end
    % If the robot is stopped then go to the next state
    if  abs(forwBackVel) < 0.005 && abs(leftRightVel) < 0.005 && abs(rotVel) < 0.01 && obj.counter > 199 && obj.reachBoolean
      obj.gripper = true;
      obj.armState = 'Lift';
      obj.counter = 0;
    end

  % Grip the object then back-off
  elseif strcmp(obj.armState,'Lift')
    obj.counter = obj.counter+1;
    if obj.liftBoolean
      liftObject(obj);
      obj.liftBoolean = false;
      obj.gripper = false; % Important otherwise out of time in testOfTables

      % The youbot can back-off
      setNewRadiusNearTable(holonomicController,obj.initialRadius);
    end
    if  abs(forwBackVel) < 0.005 && abs(leftRightVel) < 0.005 && abs(rotVel) < 0.01 && obj.counter > 199
      obj.liftBoolean = true;
      obj.counter = 0;
      obj.armState = 'placeOnYoubot';
    end

  % Once the youbot is backed-off the arm can return in its initial configuration
  elseif strcmp(obj.armState,'placeOnYoubot')
    if obj.counter == 0
      placeOnYoubot(obj);
    end
    obj.counter = obj.counter+1;
    % Wait a bit before putting the arm in its transport configuration
    if obj.counter > 200
      obj.isDone = true;
      obj.armState = 'Rotation';
      obj.currentJointsPositions = obj.transportPosition;
      obj.counter = 0;
    end
  end
  gripper = obj.gripper;
  isDone = obj.isDone;
  jointsPositions = obj.currentJointsPositions;

end % End of object pickup

function [jointsPositions, gripperRelease, isDone] = objectDrop(obj, holonomicController, forwBackVel, leftRightVel, rotVel)
  % First set get the robot in position near the table to drop
  if strcmp(obj.dropState,'Positioning')
    if obj.positioningBoolean
      setTablePosition(holonomicController, obj.tableToDrop);
      setRobotPositionNearTable(holonomicController, obj.initialRadius, obj.angularPositionToDrop);
      obj.positioningBoolean = false;
    end
    obj.counter = obj.counter+1;

    if abs(forwBackVel) < 0.005 && abs(leftRightVel) < 0.005 && abs(rotVel) < 0.01 && obj.counter > 199
      obj.dropState = 'Prelift';
      obj.positioningBoolean = true;
      obj.counter = 0;
    end

  % Then get lift the arm a little bit so that it does not interfere with the youbot's surface
  elseif strcmp(obj.dropState,'Prelift')
    if obj.counter <1
      obj.currentJointsPositions = obj.initialJointsPositions; % Originaly transportPosition
    end
    obj.counter = obj.counter+1;
    if obj.counter> 100
      obj.counter = 0; % Reset the counter
      obj.dropState = 'RotateAndLiftAndGetClose';
    end

  % Put the arm in a position wich will not interfere with the basket
  elseif strcmp(obj.dropState,'RotateAndLiftAndGetClose')
    if obj.counter <1
      obj.currentJointsPositions = obj.dropJoints;
    end
    obj.counter = obj.counter+1;
    % Wait a bit before moving the entire youbot near the basket
    if obj.counter> 200
      % Get close of the table
      setNewRadiusNearTable(holonomicController,obj.minimumRadius);
      obj.counter = 0;
      obj.dropState = 'Drop';
    end

  % Then drop the object
  elseif strcmp(obj.dropState,'Drop')
    obj.counter = obj.counter+1;
    if obj.counter > 200 && abs(forwBackVel) < 0.005 && abs(leftRightVel) < 0.005 && abs(rotVel) < 0.01
      obj.gripperRelease = true;
      obj.counter = 0;
      obj.dropState = 'backOff';
    end

  % Then back off
  elseif strcmp(obj.dropState,'backOff')
    if obj.counter < 1
      obj.gripperRelease = false; % Important otherwise out of time in testOfTables
      setNewRadiusNearTable(holonomicController, obj.initialRadius);
    end
    obj.counter = obj.counter+1;
    if obj.counter > 200 && abs(forwBackVel) < 0.005 && abs(leftRightVel) < 0.005 && abs(rotVel) < 0.01
      obj.currentJointsPositions = obj.initialJointsPositions;
      obj.isDone = true;
    end
  end % End of state machine

  gripperRelease = obj.gripperRelease;
  isDone = obj.isDone;
  jointsPositions = obj.currentJointsPositions;
end % End of objectDrop

function initialRotation(obj)
  % Simply rotate the arm of the youbot
  jointsPositions = obj.initialJointsPositions;
  jointsPositions(1) = -pi/2;
  jointsPositions(4) = 0;

  obj.currentJointsPositions = jointsPositions;

end % End of initialRotation

function  positioning(obj)

  % Get the joints positions
  % Solve the set of nonlinear equations with the matlab symbolic toolbox
  syms thetaA;
  syms thetaB;

  equation1 = 0.9*(obj.l1+obj.l2) == obj.l1*sin(thetaA)+obj.l2*sin(thetaB+thetaA);
  equation2 = obj.yPointToGrab == obj.l1*cos(thetaA)-obj.l2*cos(pi-(thetaB+thetaA));

  solution = vpasolve([equation1 equation2], [thetaA thetaB], [-1.5707963705063, 1.308996796608; 0, 2.2863812446594]); % Solution should respect the joints ranges

  if isempty(solution.thetaA)
    error('No solution to the nonlinear problem');
  end

  thetaA = double(solution.thetaA);
  thetaB = double(solution.thetaB);
  thetaC = pi/2-(thetaA+thetaB); % Third part of the arm on a horizontal line

  % Return the values
  obj.currentJointsPositions = [obj.currentJointsPositions(1), thetaA, thetaB, thetaC, obj.currentJointsPositions(5)];

  xTip = obj.l1*sin(thetaA)+obj.l2*sin(thetaA+thetaB)+obj.l3;

  obj.distTipObject = abs(obj.xPointToGrab-xTip)+(obj.initialRadius-1);

end % End of positioning

function  liftObject(obj)

  % Simply lift the object up in the air to avoid contact with any other object

  % Use the extend function
  obj.yPointToGrab = obj.yPointToGrab + obj.incr; % Move the object up
  positioning(obj);

end % End of lift

function  placeOnYoubot(obj)

  % Simply goes back to the initial joint configuration
  obj.currentJointsPositions = obj.initialJointsPositions;

end % End of placeOnYoubot

function setPointToGrab(obj, objectRadialPosition, objectAngularPosition, objectAbsoluteHeight, tablePosition, armPosition)

  % Absolute coordinates of the point to grab
  objectAbsolutePosition = [tablePosition(1)+objectRadialPosition*cos(objectAngularPosition), ...
    tablePosition(2)+objectRadialPosition*sin(objectAngularPosition)];

  % Point to grab is in a 2D plane containing the point to grab and the arm. The frame is centered at the bottom of the arm
  objectAbsolutePosition = objectAbsolutePosition-[armPosition(1) armPosition(2)]; % Center the frame to have the youbot in 0,0
  pointToGrab(1) = abs(sqrt(objectAbsolutePosition(2)^2+objectAbsolutePosition(1)^2)); % In that plane the "x" coordinate is simply the distance
  pointToGrab(2) = objectAbsoluteHeight+0.05;

  obj.xPointToGrab = pointToGrab(1);
  obj.yPointToGrab = pointToGrab(2)-obj.l0; % (0,0) on the first horizontal hinge axis

  % Reset some variables
  obj.isDone = false;
  obj.liftBoolean = true;
  obj.reachBoolean = false;


end % End of setPointToGrab

function setPointToGrabMapFrame(obj, objectAbsolutePosition, armPosition)
  % Point to grab is in a 2D plane containing the point to grab and the arm. The frame is centered at the bottom of the arm
  objectAbsolutePosition = objectAbsolutePosition-[armPosition(1) armPosition(2) 0]; % Center the frame to have the youbot in 0,0
  pointToGrab(1) = abs(sqrt(objectAbsolutePosition(2)^2+objectAbsolutePosition(1)^2)); % In that plane the "x" coordinate is simply the distance
  pointToGrab(2) = objectAbsolutePosition(3)+0.05;

  obj.xPointToGrab = pointToGrab(1);
  obj.yPointToGrab = pointToGrab(2)-obj.l0; % (0,0) on the first horizontal hinge axis

  % Reset some variables
  obj.isDone = false;
  obj.liftBoolean = true;
  obj.reachBoolean = false;
end

function [pointToPlaceYoubot, slopeAngle] = getPointToPlaceYoubot(obj, objectAbsolutePosition, tablePosition)
  objectPosition(1) = objectAbsolutePosition(1) - tablePosition(1);
  objectPosition(2) = objectAbsolutePosition(2) - tablePosition(2);

  slope = objectPosition(2)/objectPosition(1);

  slopeAngle = atan(slope);

  % Choose solution depending on the way points
  if objectPosition(2)>=0 && objectPosition(1)>=0
    pointToPlaceYoubot(1) = obj.initialRadius*cos(slopeAngle);

  elseif objectPosition(2)>=0 && objectPosition(1)<0
    pointToPlaceYoubot(1) = -obj.initialRadius*cos(slopeAngle);
    if slopeAngle>pi
      slopeAngle = slopeAngle -pi;
    else
      slopeAngle = slopeAngle +pi;
    end

  elseif objectPosition(2)<0 && objectPosition(1)>=0
    pointToPlaceYoubot(1) = obj.initialRadius*cos(slopeAngle);

  elseif objectPosition(2)<0 && objectPosition(1)<0
    pointToPlaceYoubot(1) = -obj.initialRadius*cos(slopeAngle);
    if slopeAngle>pi
      slopeAngle = slopeAngle -pi;
    else
      slopeAngle = slopeAngle +pi;
    end
  end % End if

  % Remaining coordinate
  % If infinite slope, choose the solution depending on the segment's orientation
  if slope == Inf
    pointToPlaceYoubot(2) = obj.initialRadius;
  elseif slope == -Inf
    pointToPlaceYoubot(2) = -obj.initialRadius;
  else
    pointToPlaceYoubot(2) = slope*pointToPlaceYoubot(1);
  end

  pointToPlaceYoubot = pointToPlaceYoubot + [tablePosition(1) tablePosition(2)];

end

function pointToPlaceYoubot = getPointToPlaceYoubotWithAngle(obj, angleNearTable, tablePosition)

  slope = tan(angleNearTable);

  % Choose solution depending on the way points
  if abs(angleNearTable)< pi/2
    pointToPlaceYoubot(1) = obj.initialRadius*cos(angleNearTable);
  else
    pointToPlaceYoubot(1) = -obj.initialRadius*cos(angleNearTable);
  end % End if

  % Remaining coordinate
  % If infinite slope, choose the solution depending on the segment's orientation
  if slope == Inf
    pointToPlaceYoubot(2) = obj.initialRadius;
  elseif slope == -Inf
    pointToPlaceYoubot(2) = -obj.initialRadius;
  else
    pointToPlaceYoubot(2) = slope*pointToPlaceYoubot(1);
  end

  pointToPlaceYoubot = pointToPlaceYoubot + [tablePosition(1) tablePosition(2)];

end

function setPlaceToDrop(obj, tableToDrop, angularPositionToDrop)
  obj.tableToDrop = tableToDrop;
  obj.angularPositionToDrop = angularPositionToDrop;
  obj.counter = 0;
  obj.isDone = false;
  obj.dropState = 'Positioning';
  obj.positioningBoolean = true;
end % End of set place to drop

end % End of methods

end % End of classdef
