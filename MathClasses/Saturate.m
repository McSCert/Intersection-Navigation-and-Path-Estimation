classdef Saturate
    %SATURATE - limits the maximum and minimum values for
    % an inputed value
    properties
        maximum = 100;
        minimum = -100;
    end
    
    methods

        %% Create a SATURATE object
        % OBJ = SATURATE(MIN,MAX)
        % Creates a saturate object based on the min (MIN) and 
        % max (MAX) values given
        function obj = Saturate(min,max)
            %SATURATE Construct an instance of this class
            %   Detailed explanation goes here
            obj.minimum = min;
            obj.maximum = max;
        end
        

        %% Limit the value
        % LIMITED_VALUE = LIMIT(OBJ,VALUE)
        % Limits the input value (VALUE) based on the objects min and max values 
        %
        %
        % OUTPUT
        %
        % LIMITED_VALUE - the limited value
        function limited_value = limit(obj,value)
            limited_value = min(max(value,obj.minimum),obj.maximum);
        end
    end
end

