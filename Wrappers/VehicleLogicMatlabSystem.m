classdef VehicleLogicMatlabSystem < matlab.System
    % VEHICLELOGICMATLABSYSTEM
    %
    % A Wrapper Matlab system that is used within the vehicle system block
    % in the simulink simulation

    % Public, tunable properties
    properties
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        
    end
    methods (Access = public)

        %% Initialize The values in the matlab system block
        % INIT(OBJ) - updates and calls the setup implemation
        % function
        function init(obj)
            setupImpl(obj);
        end
        
        %% Update The values in the matlab system block
        % UPDATE(OBJ,  FRONT_POS, REAR_POS, VELOCITIES, YAWS)
        % updates the positions of the vehicles based on the front
        % and rear positions of the vehicles (FRONT_POS, REAR_POS)
        % the velocities and yaws of the vehicle (VELOCITIES, YAWS)
        function [target_speed, accel, streering_angle] = update(obj,  front_pos, rear_pos, velocities, yaws)
            [target_speed, accel, streering_angle] = stepImpl(obj, front_pos, rear_pos, velocities, yaws);
        end
    end
    methods(Access = protected)
        %% Initialize The values in the matlab system block
        % SETUPIMPL(OBJ) - updates and calls the wrapper initialize function
        % function
        function setupImpl(obj)
            coder.extrinsic('Wrapper_CallInitSim'); 
            Wrapper_CallInitSim();
            
        end

        %% Update The values in the matlab system block
        % [TARGET_SPEED,ACCEL, STREERING_ANGLE] = STEPIMPL(OBJ,  FRONT_POS, REAR_POS, VELOCITIES, YAWS)
        % updates the positions of the vehicles based on the front
        % and rear positions of the vehicles (FRONT_POS, REAR_POS)
        % the velocities and yaws of the vehicle (VELOCITIES, YAWS)
        %
        % OUTPUTS
        %
        % TARGET_SPEED - The target speed of the vehicle
        % ACCEL - The next step to acceleration of the vehicle
        % STREERING_ANGLE - the next step to steering angle of the vehicle
        function [target_speed,accel, streering_angle] = stepImpl(obj, front_pos, rear_pos, velocities, yaws)
            coder.extrinsic('Wrapper_CallUpdatePositions');
            coder.extrinsic('Wrapper_CallUpdateMessages'); 
            %debug_command = (['[target_speed,accel, streering_angle] = vvv.update(['...
            %    int2str(positions) '] , [' int2str(velocities) '])']);
            total_vehicles = 2;
            
            %target_speed = zeros(1,length(total_vehicles));
            %streering_angle = zeros(1,length(total_vehicles)); 
            %accel = zeros(1,length(total_vehicles));
            target_speed = [0 0];
            streering_angle = [0 0];
            accel = [0 0];
            for i = 1:1:total_vehicles
                
                I = Wrapper_CallUpdatePositions(i,  front_pos, rear_pos, velocities, yaws);
                %idims = zeros(1,3); %Need to preallocate idims so it does not become an mxArray
                %Itemp
                %debug = size(Itemp)
                %idims = size(Itemp);
                %I = coder.nullcopy(zeros(idims, 'double'));  % Allocate but do not initialize an array of double values of the same size as Itemp
                %I = Itemp; % Copy the data from the mxArray into the final variable.
                target_speed(i) = I(1);
                accel(i) = I(2);
                streering_angle(i) = I(3); 
                
                
            end
            Wrapper_CallUpdateMessages();
            
            %target_speed = target_speed(1);
            %streering_angle = streering_angle(1); 
            
            
        end

        %% Reset The values in the matlab system block
        % RESETIMPL(OBJ) - resets the matlab system block variables
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
