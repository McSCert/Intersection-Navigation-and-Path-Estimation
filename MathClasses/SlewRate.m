classdef SlewRate
    %SLEWRATE slews the inputed values based on a max rate up and 
    % down to prevent any sharp increases/decreases
    
    properties
        max_rate_up;
        max_rate_down;
        previous_value;
    end
    
    methods

        %% Create a SLEWRATE object
        % OBJ = SLEWRATE(MAX_RATE_DOWN, MAX_RATE_UP , START_VALUE)
        % Creates a slew rate object based on the max rate down (MAX_RATE_DOWN) 
        % and the max rate up (MAX_RATE_UP). The inital value of the object is
        % START_VALUE
        function obj = SlewRate(max_rate_down, max_rate_up , start_value)
            obj.max_rate_down = abs(max_rate_down);
            obj.max_rate_up = abs(max_rate_up);
            obj.previous_value = start_value;
        end
        

        %% Limit the value based on the slew rates
        % [OBJ, LIMITED_VALUE] = SLEW(OBJ,NEW_VALUE)
        % Limits the input value (NEW_VALUE) based on the objects max up/down
        % slew rates 
        %
        % OUTPUT
        %
        % LIMITED_VALUE - the limited value
        %
        function [obj, limited_value] = slew(obj,new_value)
            if new_value < obj.previous_value - obj.max_rate_down
                limited_value = obj.previous_value - obj.max_rate_down;
            elseif obj.previous_value + obj.max_rate_up < new_value
                limited_value = obj.previous_value + obj.max_rate_up;
            else
                limited_value = new_value;
            end
            obj.previous_value = limited_value;
        end
    end
end

