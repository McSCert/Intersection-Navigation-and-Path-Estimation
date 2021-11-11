classdef VehicleLogicSimulator
    % VEHICLELOGICSIMULATOR
    %
    % A Matlab system that is used within the vehicle system block
    % in the simulink simulation to simulate the logic of the 
    % vehicles
        
    


    % Pre-computed constants
    properties(Access = protected)
        vehicle_objs;
        vehicle_goals;
        vIDs;
        map;
        message_channel_obj;
    end

    methods (Access = public)

        %% Initialize The values in the vehicle simulation
        % INITSIM(OBJ) - sets up the nessary variables for the vehicle
        % logic
        function obj = InitSim(obj)
            % Perform one-time calculations, such as computing constants
            total_vehicles = evalin('base','total_vehicles');
            obj.vehicle_objs= cell(1,total_vehicles);
            obj.vIDs = evalin('base','vid');
            
            %Paths for the nodes
            obj.vehicle_goals = evalin('base','vehicle_nodes');
            
            %Current X,Y positions
            x0 = evalin('base','x0');
            y0 = evalin('base','y0');
            %{[6 4.5];
            %[57 1.5]};
            
            yaw0 = evalin('base','yaw0');
            L_yaw = evalin('base','L_yaw');
            speed_limit = evalin('base','vehicle_speed_limit');
            turn_limit = evalin('base','vehicle_turn_speed_limit');
            
             loaded_map = load('MarcCityGraphObj.mat');%Map('MarcCity.mat', 10e-3);
             obj.map = loaded_map.marc_city;
             obj.message_channel_obj = Message_Medium(obj.vIDs);
             for i = 1:1:total_vehicles
                 path= obj.vehicle_goals{i};
                 
                 av = Autonomous_Vehicle('SimulationModel', obj.vIDs(i), ...
                     obj.message_channel_obj, obj.map, speed_limit(i), turn_limit(i), ...
                     path, [y0(i) x0(i)], L_yaw, ((180.0*yaw0(i))/pi));
                 obj.vehicle_objs{i} =  av;
             end
        end

        %% Update the positions of the vehicles
        % [TARGET_SPEED, ACCEL, STREERING_ANGLE] = UPDATEPOSITIONS(OBJ, INDX, FRONT_POS, REAR_POS, VELOCITIES, YAWS)
        % updates the positions of the vehicle with ID (INDX), based on the current vehicle
        % positions (FRONT_POS, REAR_POS) and vehicle velocity and yaw angle (VELOCITIES, YAWS)
        function [target_speed, accel, streering_angle] = UpdatePositions(obj, indx, front_pos, rear_pos, velocities, yaws)
            
           %obj.message_channel_obj.updateMessages('SimulationModel',obj.vehicle_objs);
           
            %Updating the states of the vehicle objects
             fx = front_pos((indx-1)*length(obj.vIDs) + 2);
             fy = front_pos((indx-1)*length(obj.vIDs) + 1);
             bx = rear_pos((indx-1)*length(obj.vIDs) + 2);
             by = rear_pos((indx-1)*length(obj.vIDs) + 1);
             obj.vehicle_objs{indx}.updateState([fx, fy], [bx, by] , velocities(indx),yaws(indx));
 
            
             streering_angle = obj.vehicle_objs{indx}.get_steering_angle();
             target_speed = obj.vehicle_objs{indx}.get_trgt_spd();
             accel = obj.vehicle_objs{indx}.get_accel();
        end
        
        %% Update the messages of the vehicles
        % UPDATEMESSAGES
        % Updates the messages that are currently in transmission
        function UpdateMessages(obj)
           obj.message_channel_obj.updateMessages(obj.vehicle_objs) 
        end

        %% Reset The values in the matlab system block
        % RESETIMPL(OBJ) - resets the matlab system block variables
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
