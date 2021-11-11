classdef Channel
    %CHANNEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access=private)
        frequency
        timers
        step
        message
    end
    
    methods

        %% Create channel frequency object
        % OBJ = CHANNEL(FREQUENCY, NUMOFACTORS)  
        %
        function obj = Channel(frequency, numOfActors)
            obj.frequency = frequency;
            
            obj.timers = zeros(numOfActors,1); 
            for i = 1:1:numOfActors
               obj.timers(i) =  -Inf;
            end
            obj.step = evalin('base','Ts');   
            obj.message = '';
        end
        

        %% Get the frequency of the channel
        % OUTPUT = GETFREQUENCY(OBJ) - Return the frequency of channel
        %
        function output = getFrequency(obj)
            output = obj.frequency;
        end
        
        %% Get the current message that is being sent on the  channel
        % OUTPUT = GETMESSAGE(OBJ) - Return the current message that is being sent on the  channel
        %
        function output = getMessage(obj)
            output = obj.message;
        end
        
        %% Get the message times
        % OUTPUT = GETTIMER(OBJ) - Return the amount of time left before the entire message
        % has been sent to each vehicle
        %
        function output = getTimer(obj, vID)
            output = obj.timers(vID);
        end
        
        %% Get the message timer for a specific vehicle
        %  [OBJ, TIME] = UPDATETIMER(OBJ, VID) update the timer for a vehicle based on the 
        % vehicle ID (VID) and returns the current time (TIME) for that vehicle
        %
        function [obj,time] = updateTimer(obj,vID)
             obj.timers(vID) =  obj.timers(vID) - obj.step;
             time = obj.timers(vID);
        end
        
        %% Reset a timer
        %  [OBJ] = RESETTIMER(OBJ,VID) - reset the timer for a specific vehicle based on the
        % vehicle ID (VID)
        % 
        function [obj] = resetTimer(obj,vID)
             obj.timers(vID) =  -Inf;
        end
        

        %% Broadcast a message on the channel
        %
        % OBJ = BROADCASTMESSAGE(OBJ, MESSAGE, TIMERS) broadcast a message (MESSAGE) 
        % on the channel with specific timers for each vehicle (TIMERS)
        % 
        function obj = broadcastMessage(obj, message, timers)
            obj.message = message;
            
            
            for i= 1:1:length(timers)
               obj.timers(i) = timers(i); 
            end
        end
        
        %% Check if the channel is empty
        %  BOOL = ISCHANNELEMPTY(OBJ) returns if there are vehicles 
        %  that are still waiting for the message
        function bool = isChannelEmpty(obj)
           bool =true;
           for i = 1:1:length(obj.timers) 
                bool = bool && (obj.timers(i) == -Inf);    
           end
           
        end
    end
end

