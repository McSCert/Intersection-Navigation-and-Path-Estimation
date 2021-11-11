classdef Message_Medium < matlab.DiscreteEventSystem
    %MESSAGE_CHANNEL The set of channel frequencys that the vehicles can communicate off of
    % has various queues for each of the different channels to allow message passing
    
    properties
        numOfVehicles
        channels ={};
        vehicleIDs = [];
        min_channel = 5.85e9;
        max_channel = 5.925e9;
        lowRange = 10
        hiRange = 11;
        band_size = 10e6;
        padding_front=5e6;
        padding_back=0;
        control_channel = 1;
        num_of_channels =7;
        static_channel = true;

        %% Delay properties
        T_p = 32*(10e-6);
        T_phy = 64*(10e-6);
        T_sym = 8*(10e-6);
        tau = 2*(10e-6);
        N_dbps_data = 24;
        LH_data = 28;
    end
    
    methods

        %% Create message medium object
        % OBJ = MESSAGE_MEDIUM(VEHICLEIDS)
        %
        function obj = Message_Medium(vehicleIDs)
            %MESSAGE_CHANNEL Construct an instance of this class
            %   Detailed explanation goes here
            rng('shuffle');
            obj.numOfVehicles = length(vehicleIDs); 
            
            obj.channels = {
                 Channel(obj.min_channel + obj.band_size*(0), length(vehicleIDs))...
                 Channel(obj.min_channel + obj.band_size*(1), length(vehicleIDs))...
                 Channel(obj.min_channel + obj.band_size*(2), length(vehicleIDs))...
                 Channel(obj.min_channel + obj.band_size*(3), length(vehicleIDs))...
                 Channel(obj.min_channel + obj.band_size*(4), length(vehicleIDs))...
                 Channel(obj.min_channel + obj.band_size*(5), length(vehicleIDs))...
                 Channel(obj.min_channel + obj.band_size*(6), length(vehicleIDs))
                      };
            %for i=1:1:obj.num_of_channels
            %    obj.channels{i} = Channel(obj.min_channel + obj.band_size*(i-1), length(vehicleIDs));
            %end
            obj.vehicleIDs = vehicleIDs;
        end
        
        %% add a vehicle to the set of vehicles in the simulation
        %
        % ADDVEHICLE(OBJ, VEHICLE) register a vehicle to recieve messages sent
        % on the message channel
        function addVehicle(obj, vehicle)
            %ADD Vehicle - Addes a vehicle to the set of vehicles that will
            %recieve messages
            obj.numOfVehicles = obj.numOfVehicles+1;
            obj.messageQueue(end) = vehicle.getMessageQueue();
            obj.vehicleIDs(end) = vehicle.getVehicleID();
            
        end
        
        %% Send a message to the vehicles
        %
        % SENDMESSAGE(OBJ,FROMVEHICLEID, MESSAGE, CHANNEL) send a message (MESSAGE)
        % from a vehicle (with vehicle ID FROMVEHICLEID) on a channel number (CHANNEL)
        %
        function sendMessage(obj,fromVehicleID, message, channel)
            %SEND_MESSAGE - sends the json text to each of the different
            %vehicles
            positions = evalin('base', 'vehicle_positions');
            timers = zeros(obj.numOfVehicles, 1);
            for i = 1:1:obj.numOfVehicles
                timers(i)=-Inf;
                if i ~= fromVehicleID
                    randNum = rand(1)*(10e6 -1) +1;
                    if ~(obj.lowRange <= randNum && randNum <= obj.hiRange)
                        from_pos =  [ positions((fromVehicleID-1)*length(obj.numOfVehicles) + 2), ...
                            positions((fromVehicleID-1)*length(obj.numOfVehicles) + 1)];
                        to_pos =  [ positions((i-1)*length(obj.numOfVehicles) + 2), ...
                            positions((i-1)*length(obj.numOfVehicles) + 1)];
                        
                        
                        timers(i) = obj.calculateDelay(from_pos, to_pos, channel, message);
                       
                    end
                    
                end
                
            end
            obj.channels{obj.control_channel}= ...
                obj.channels{obj.control_channel}.broadcastMessage(message, timers);
            
        end
        
        %% Calculate the delay to a vehicle
        %
        % TIMER = CALCULATEDELAY(OBJ, FROM_POS, TO_POS, CHANNEL_NUM, MESSAGE) calculate
        % the delay for a vehicle for the (MESSAGE)
        function timer = calculateDelay(obj, from_pos, to_pos, channel_num, message)
            %channel = obj.channels{channel_num};
            %x1 = from_pos(1);
            %y1 = from_pos(2);
            
            %x2 = to_pos(1);
            %y2 = to_pos(2);
            %dist = sqrt((y2 -y1)^2 + (x2-x1)^2);
            %(dist/(299792458))

            message_length = (obj.LH_data + length(message));
            timer =  obj.tau + obj.T_p + obj.T_phy + ...
                obj.T_sym*ceil((16 + 6 + 8*message_length)/obj.N_dbps_data);
            
        end
        

        %% Update the timers for the vehicles
        %
        % FUNCTION UPDATEMESSAGES(OBJ, VEHICLES_OBJS) update each of the timmers for the
        % vehicles (VEHICLES_OBJS). When the timer reaches 0, send the message and
        % reset the timer
        function updateMessages(obj, vehicles_objs)
            
            for i = 1:1:obj.numOfVehicles
               [obj.channels{obj.control_channel},time] = obj.channels{obj.control_channel}.updateTimer(i); 
               
               message = obj.channels{obj.control_channel}.getMessage();
               if time < 0 && -inf < time && ~isempty(message)
                   vehicles_objs{i}.recieve_message(message);
                   obj.channels{obj.control_channel}= obj.channels{obj.control_channel}.resetTimer(i);
               end
               
            end
            
        end
        
        
        %% Check to see if the control channel is free
        %
        % OUTPUT = ISCONTROLCAHNNELFREE(OBJ) provides a boolean flag if the control channel
        % does not currently have a message
        %
        function output = isControlCahnnelFree(obj)
            output = obj.channels{obj.control_channel}.isChannelEmpty(); 
            
            
        end
        

        %% Get the control channel object
        %
        % OUTPUT = GETCONTROLCHANNEL(OBJ) provides an object of the control channel
        %
         function output = getControlChannel(obj)
            output = obj.control_channel;   
        end
    end
   
    
    methods (Access=private)

        %% Get the control channel object
        %
        % INDEX = GETCHANNELINDX(OBJ, FREQ) Calculates the channel INDEX based on the
        % freqency (FREQ)
        function index = getChannelIndx(obj, freq)
            index = floor((freq -  (obj.min_channel+ obj.padding_front))/obj.band_size);
        end
        
    end
    
end

