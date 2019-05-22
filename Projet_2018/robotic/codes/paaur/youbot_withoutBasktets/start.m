if exist('loaded') == 0 % Loads RC toolbox if not done yet
    run('../matlab/startup_robot.m');
    loaded = true;
end
close all; 


%dashboard_160; % Open GUI with resolution of 10 cm 
dashboard_80; % Open GUI with resolution of 20 cm 