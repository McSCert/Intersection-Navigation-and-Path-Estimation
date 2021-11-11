classdef Vehicle <  matlab.DiscreteEventSystem
    %% properties
    properties
    end

        % Pre-computed constants
    properties(Access = protected)
        initVelocity=10; % Start Speed (Km/h):
        initAcc=10; % Seconds 0-100 Km/h:         
        initLane=1; % Initial Lane (1-4):       
        startPositionX=10; % Initial PositionX:
        startPositionY=0;  % Initial PositionY:
              
        curPositionX; % current position
        curPositionY;
        curSpeedX %current speed
        curSpeedY
        acceleration=0;
    end

    methods(Access = protected)   


    end
end