function measurement = HelperCarBotGetGPSReading(carbot, sd)
%   HELPERCARBOTGETGPSREADING Generate a mock GPS reading of the robot
%   pose. Here the GPS reading is just simulated by adding Gaussian Noise
%   to the ground truth data. Change the noise
%   standard deviation (sd) to see how it affects the tracking behavior. 

%   Copyright 2015 The MathWorks, Inc.
    
    pose = carbot.Pose;
    measurement = pose(1:3) + sd*randn(3,1);
    
    if carbot.isCarInRoofedArea
        measurement = [];
    end
    
end