function [ranges, angles] = sensor(vrep, h, opmode, id_sensor, x)
% Reads from Hokuyo sensor.
  
    if id_sensor == 1
        [~, ~, auxData, auxPacketInfo] = vrep.simxReadVisionSensor(h.id, h.hokuyo1, opmode);
    else
        [~, ~, auxData, auxPacketInfo] = vrep.simxReadVisionSensor(h.id, h.hokuyo2, opmode);
    end

    % The Hokuyo data comes in a funny format. Use the code below to move it
    % to a Matlab matrix
    width = auxData(auxPacketInfo(1)+1);
    height = auxData(auxPacketInfo(1)+2);
    pts = reshape(auxData((auxPacketInfo(1)+2+1):end), 4, width*height); % Each column of pts has [x;y;z;distancetosensor]
    ranges = pts(4,:);
    
    % Each sensor has a breadth range of 120 degrees and resolution of 256
    n_measures = size(ranges, 2);
    if id_sensor == 1
        angles = linspace(5*pi/6, 2*pi/3+5*pi/6, n_measures); % Sensor is 60 degrees counter cw from center  
    else
        angles = linspace(-3*pi/6, 2*pi/3-3*pi/6, n_measures); % Sensor is 60 degrees cw from center
    end
end
