classdef PID
    %PID controller class
    
    properties
        Kp = 1.5;
        Ki = 0.05;
        Kd = 0;
        intgl_error = 0;
        previous_error = 0;
        Ts = 0.05;
    end
    
    methods

        %% Create a pid controller object
        % OBJ = PID(KP, KI, KD)
        % Creates a pid controller object with the proportional gain being KP,
        % the integral term being KI and the derivative term being KD
        function obj = PID(Kp,Ki,Kd)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
        end
        
        %% Perform PID step
        % [OBJ,OUTPUT] = PERFORM_PID(OBJ,REF, FDBK)
        % Performs the PID step based on the feedback (FDBK) and reference (REF)
        %
        %
        % OUTPUT
        %
        % output - the output error based on Kp*error + Ki*intgl_error+Kd*drvt_error
        function [obj,output] = perform_PID(obj,ref, fdbk)
            % CONTROL LOOP
            error        = ref- fdbk;                        % update error
            obj.intgl_error = obj.intgl_error + error*obj.Ts;     % integral term
            drvt_error   = (error - obj.previous_error)/obj.Ts;  % derivative term
            output    = obj.Kp*error+obj.Ki*obj.intgl_error+obj.Kd*drvt_error; % action of control
            obj.previous_error = error;
        end
        

        %% Perform PID step
        % [OBJ,OUTPUT] = PERFORM_PID(OBJ,REF, FDBK)
        % Performs the PID step based on the error (ERROR) from the feedback and reference values
        %
        %
        % OUTPUT
        %
        % output - the output error based on Kp*error + Ki*intgl_error+Kd*drvt_error
        function [obj,output] = perform_PID_wth_error(obj, error)
            % CONTROL LOOP
            obj.intgl_error = obj.intgl_error + error*obj.Ts;     % integral term
            drvt_error   = (error - obj.previous_error)/obj.Ts;  % derivative term
            output    = obj.Kp*error+obj.Ki*obj.intgl_error+obj.Kd*drvt_error; % action of control
            obj.previous_error = error;
        end
    end
end

