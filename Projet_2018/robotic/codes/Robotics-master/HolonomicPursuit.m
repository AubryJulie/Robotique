classdef HolonomicPursuit <handle

%===============================================================================

properties
% Lookahead distance. Default = 1
lookahead = 1; % [m]

% Different kind of speeds. Default = 1
maxLongitudinalSpeed = 1; %[m/s]
maxTransversalSpeed = 1; %[m/s]
end % End of properties

properties (Access = private)
% Path to follow. Default =  []
pathPoints;

% Angle to orient the robot
desiredOrientation;

% Position of the robot. Default = 0
position = [0 0 0]; %[m m rad]
lookaheadPoint = [0 0]; %[m m]

% Variables to make the robot start smoothly
acceleration = 0;
maxSpeedReached = 100;

% Variables to make the robot stop smoothly
deceleration = 0;
stopped = 100;

% Variables to place the robot near a table
tablePosition;
robotPositionNearTable;
radiusRelativeToTable;
angleRelativeToTable;

end % End of private properties

%===============================================================================

methods

% Constructor
function obj = HolonomicPursuit(pathPoints, lookahead, maxLongitudinalSpeed, maxTransversalSpeed)

  obj.pathPoints = pathPoints;
  obj.lookahead = lookahead;
  obj.maxLongitudinalSpeed = maxLongitudinalSpeed;
  obj.maxTransversalSpeed = maxTransversalSpeed;
end % End of constructor

% Step function
function [longitudinalSpeed, transversalSpeed, rotationSpeed] = step(obj, position)

  % Update of the position
  obj.position = position;

  % Find closest point of the robot
  distance = sqrt((obj.pathPoints(:,1) - obj.position(1)).^2 + (obj.pathPoints(:,2) - obj.position(2)).^2);
  [~,closestIndex] = min(distance);

  alreadyTravelled = 0;

  % If path is long enough, align the robot with the path, otherwise do not do that
  if length(obj.pathPoints)>4
    % Rotation speed to align with the path
    % Angle between the robot's position and the target point
    if closestIndex>= length(obj.pathPoints)-3
      directionSlope = atan2((obj.pathPoints(end,2) - obj.pathPoints(end-1,2)),(obj.pathPoints(end,1) - obj.pathPoints(end-1,1)));
    else
      % Start to rotate earlier so that the robot has time to turn otherwise it aligns itself with the end of the path
      directionSlope = atan2((obj.pathPoints(closestIndex+4,2) - obj.pathPoints(closestIndex+3,2)),(obj.pathPoints(closestIndex+4,1) - obj.pathPoints(closestIndex+3,1)));
    end

    % Rotation speed
    rotationSpeed = robotics.internal.angdiff(obj.position(3), directionSlope);

    if abs(abs(rotationSpeed) - pi) < 1e-12
      rotationSpeed = sign(rotationSpeed)*1;
    end
  else
    rotationSpeed = 0;
  end % End of if length

  % If closest point is the last of pathPoints then last of pathPoints is lookaheadPoint
  obj.lookaheadPoint = obj.pathPoints(end,:);

  % Compute the target point based on the lookahead
  while closestIndex < length(obj.pathPoints) && size(obj.pathPoints,1)>1

    % If the lookaheadPoint is somewhere on the current interval of the path
    if sqrt((obj.pathPoints(closestIndex,1)-obj.pathPoints(closestIndex+1,1))^2+...
      (obj.pathPoints(closestIndex,2)-obj.pathPoints(closestIndex+1,2))^2) > (obj.lookahead - alreadyTravelled)

      % Find the interpolated point corresponding to the lookahead distance
      slope = (obj.pathPoints(closestIndex+1,2)-obj.pathPoints(closestIndex,2))/...
      (obj.pathPoints(closestIndex+1,1)-obj.pathPoints(closestIndex,1)); % Slope of the segment
      intermediateValue  = obj.pathPoints(closestIndex,1)^2 - (obj.lookahead^2/(1+slope^2));

      % Choose solution depending on the way points
      if (obj.pathPoints(closestIndex+1,1)-obj.pathPoints(closestIndex,1))>0
        obj.lookaheadPoint(1) = obj.pathPoints(closestIndex,1) + sqrt(obj.pathPoints(closestIndex,1)^2-intermediateValue);
      else
        obj.lookaheadPoint(1) = obj.pathPoints(closestIndex,1) - sqrt(obj.pathPoints(closestIndex,1)^2-intermediateValue);
      end % End if

      % Remaining coordinate
      % If infinite slope, choose the solution depending on the segment's orientation
      if slope == Inf
        obj.lookaheadPoint(2) = (obj.lookahead - alreadyTravelled) + obj.pathPoints(closestIndex,2);
        break;
      elseif slope == -Inf
        obj.lookaheadPoint(2) = -(obj.lookahead - alreadyTravelled) + obj.pathPoints(closestIndex,2);
        break;
      end
      obj.lookaheadPoint(2) = slope*(obj.lookaheadPoint(1)-obj.pathPoints(closestIndex,1))+obj.pathPoints(closestIndex,2);
      break; % Goes out of the loop

    else

      % Go to the next interval
      alreadyTravelled = alreadyTravelled + sqrt((obj.pathPoints(closestIndex,1)-obj.pathPoints(closestIndex+1,1))^2+...
      (obj.pathPoints(closestIndex,2)-obj.pathPoints(closestIndex+1,2))^2);
      closestIndex = closestIndex+1;

    end % End if
  end % End while

  % Angle between the robot's position and the target point
  directionSlope = atan2((obj.lookaheadPoint(2) - obj.position(2)),(obj.lookaheadPoint(1) - obj.position(1)));
  mainSpeed = robotics.internal.angdiff(obj.position(3), directionSlope);

  % Holonomic speeds are derived by projecting the main speed on the robot's axis
  % Smooth acceleration
  if obj.acceleration < obj.maxSpeedReached
    longitudinalSpeed = cos(mainSpeed)*obj.maxLongitudinalSpeed*obj.acceleration/obj.maxSpeedReached;
    transversalSpeed = sin(mainSpeed)*obj.maxTransversalSpeed*obj.acceleration/obj.maxSpeedReached;
    obj.acceleration = obj.acceleration+1;
  else
    longitudinalSpeed = cos(mainSpeed)*obj.maxLongitudinalSpeed;
    transversalSpeed = sin(mainSpeed)*obj.maxTransversalSpeed;
  end

  % Correct orientation for the longitudinalSpeed
  longitudinalSpeed = -1*longitudinalSpeed;

end % End of step

% Function to stop the robot
function [longitudinalSpeed, transversalSpeed, rotationSpeed, isStopped] = stopRobot(obj,position)
  % isStopped is normally false
  isStopped = false;

  % Get the normal speeds
  [longitudinalSpeed, transversalSpeed, rotationSpeed] = step(obj,position);

  % Then decelerate
  longitudinalSpeed = longitudinalSpeed*(1-obj.deceleration/obj.stopped);
  transversalSpeed = transversalSpeed*(1-obj.deceleration/obj.stopped);
  rotationSpeed = rotationSpeed*(1-obj.deceleration/obj.stopped);
  obj.deceleration = obj.deceleration+1;

  if obj.deceleration >= obj.stopped
    % Stop the robot
    longitudinalSpeed = 0;
    transversalSpeed = 0;
    rotationSpeed = 0;

    isStopped = true;
  end % End if deceleration

end % End of stop robot

% Function to destuck the robot
function [longitudinalSpeed, transversalSpeed, rotationSpeed, isStopped] = destuck(obj)

  % Same procedure as stopRobot but without a position refresh and with opposite speeds
  [longitudinalSpeed, transversalSpeed, rotationSpeed, isStopped] = stopRobot(obj,obj.position);
  longitudinalSpeed = -1*longitudinalSpeed;
  transversalSpeed = -1*transversalSpeed;
  rotationSpeed = -1*rotationSpeed;

end % End of destuck

% Function to rotate the robot at rest
function [longitudinalSpeed, transversalSpeed, rotationSpeed, isStopped] = rotate(obj,position)

  isStopped = false;

  longitudinalSpeed = 0;
  transversalSpeed = 0;

  % Rotation speed equals to angle's difference
  rotationSpeed = robotics.internal.angdiff(position(3)+pi/2, obj.desiredOrientation);

  if (abs(angdiff(obj.desiredOrientation, position(3)+pi/2)) < .1 / 180 * pi) && ...
    (abs(angdiff(obj.position(3), position(3))) < .01 / 180 * pi)
    rotationSpeed = 0;
    isStopped = true;
  end

  % Refresh position
  obj.position = position;

end % End of rotate

% Function to stay in position near a table
function [longitudinalSpeed, transversalSpeed, rotationSpeed] = stayNearTable(obj,position)
  % The position of the robot (x,y) is given by robotPositionNearTable(1:2)
  % The orientation of the robot is such that its longitudinal axis is tangent to the circle of center tablePosition
  % and of radius dist(tablePosition,robotPositionNearTable)

  desiredRobotDirection = obj.angleRelativeToTable-pi/2;

  % Rotation speed equals to angle's difference
  rotationSpeed = robotics.internal.angdiff(position(3), desiredRobotDirection);

  % Use the speeds from the controller (Except the rotation speed)
  [longitudinalSpeed, transversalSpeed, ~] = step(obj, position);

  % Go slowly when rotating quickly
  if abs(rotationSpeed)>pi/20
    longitudinalSpeed = 0;
    transversalSpeed = 0;
  else
    longitudinalSpeed = (1-abs(rotationSpeed)/(pi/20))*longitudinalSpeed; % pi/20 is the max rotation speed allowed
    transversalSpeed = (1-abs(rotationSpeed)/(pi/20))*transversalSpeed;
  end

  % Those speeds are linked to the distance to the robotPositionNearTable (to decelerate)
  distanceBetweenPoints = sqrt((position(1)-obj.robotPositionNearTable(1))^2+(position(2)-obj.robotPositionNearTable(2))^2);
  longitudinalSpeed = longitudinalSpeed*distanceBetweenPoints;
  transversalSpeed = transversalSpeed*distanceBetweenPoints;


  % Refresh position
  obj.position = position;

end % End of stayNearTable

% Function to change the position near a table
function [longitudinalSpeed, transversalSpeed, rotationSpeed, isStopped] = changeAngleNearTable(obj,position)
  % isStopped normally false
  isStopped = false;

  % Update the robot's angle relative to table
  % The desired direction is given by the opposite of the inverse of the slope of the line connecting the radius and the robot's position
  if obj.tablePosition(2)<position(2) % Choose the right solution of the atan problem to keep the same side of the robot the closest of the table all along a rotation around a table
    obj.angleRelativeToTable = atan(-1*(position(1)-obj.tablePosition(1))/(position(2)-obj.tablePosition(2)))+pi/2;
  else
    obj.angleRelativeToTable = atan(-1*(position(1)-obj.tablePosition(1))/(position(2)-obj.tablePosition(2)))-pi/2;
  end

  % Get the speeds from stayNearTable (The path points are such that the goes around the table)
  [longitudinalSpeed, transversalSpeed, rotationSpeed] = stayNearTable(obj,position);
  if sqrt((position(1)-obj.robotPositionNearTable(1))^2+(position(2)-obj.robotPositionNearTable(2))^2)<0.1 && abs(abs(position(3)-obj.angleRelativeToTable)-pi/2)<0.1*pi/180
    isStopped = true;
    setRobotPositionNearTable(obj,obj.radiusRelativeToTable,obj.angleRelativeToTable);
  end
end % End of changePositionNearTable

function setDesiredOrientation(obj,angle)
  obj.desiredOrientation = angle;
end % End of setDesiredOrientation

function setPathPoints(obj,pathPoints)
  obj.pathPoints = pathPoints;
  obj.acceleration = 0; % Reset the acceleration variable
  obj.deceleration = 0; % Reset the braking variable
end % End of setPathPoints

function booleanAnswer = isPathEmpty(obj)
  booleanAnswer = isempty(obj.pathPoints);
end % End of isPathEmpty

function setTablePosition(obj,tablePosition)
  obj.tablePosition = tablePosition;
end % End of setTablePosition

function setRobotPositionNearTable(obj,radius,angle)
  positionNearTable(1) = radius*cos(angle)+obj.tablePosition(1);
  positionNearTable(2) = radius*sin(angle)+obj.tablePosition(2);
  obj.radiusRelativeToTable = radius;
  obj.angleRelativeToTable = angle;
  obj.robotPositionNearTable = positionNearTable;
  setPathPoints(obj,positionNearTable); % To use the already implemented function step
end % End of setRobotPositionNearTable

function setNewAngleNearTable(obj,newAngle)

  % Generate the path points to follow
  angleSample  = linspace(obj.angleRelativeToTable,newAngle,ceil(abs(obj.angleRelativeToTable-newAngle)/0.01)); % 1 point every 0.01rad
  newX = obj.radiusRelativeToTable*cos(angleSample)+obj.tablePosition(1);
  newY = obj.radiusRelativeToTable*sin(angleSample)+obj.tablePosition(2);
  setPathPoints(obj,[newX;newY]');

  % The new positionNearTable is
  obj.robotPositionNearTable = [newX(end);newY(end)]';

end  % End of setNewAngleNearTable

function setNewRadiusNearTable(obj,newRadius)
  positionNearTable(1) = newRadius*cos(obj.angleRelativeToTable)+obj.tablePosition(1);
  positionNearTable(2) = newRadius*sin(obj.angleRelativeToTable)+obj.tablePosition(2);
  obj.radiusRelativeToTable = newRadius;
  obj.robotPositionNearTable = positionNearTable;
  setPathPoints(obj,positionNearTable); % To use the already implemented function step
end % End of setNewRadiusNearTable

function changeMaxVelocity(obj,maxLongitudinalSpeed,maxTransversalSpeed)
    obj.maxLongitudinalSpeed = maxLongitudinalSpeed;
    obj.maxTransversalSpeed = maxTransversalSpeed;
end % End of changeMaxVelocity

end % End of methods

end % End of classdef
