classdef Autonomous_Vehicle < Vehicle
    %% properties
    properties
        
    end
    
    properties(Access = private)
        entity_counter % the 1st entity is to trigger the movement of the car
        start_time   % moment when the car enters the road
        
        % constants
        model_name;
        speed_limit=3;
        turn_limit=1;
        turn_spd = 2;
        min_thresh_to_nxt_vertex = 0.75;%m 0.75
        vehicle_id= 1;
        L_axle = 1;
        use_est_pos= false;
        feedback_pos_write_buffer = 20;
        est_pos_write_buffer = 100;
        feedback_pos_file_names = '';
        estimated_pos_file_names = '';
        
        %Map/Path
        map;
        path;
        %Path
        estimated_path;
        feedback_path;
        
        %State of Car
        vehicle_plan = Vehicle_Movement.Driving_Straight;
        vehicle_curr_trgt_dir = Dir_Enum.Est
        vehicle_trgt_dir = Dir_Enum.Est;
        curr_x_rear = 0;
        curr_y_rear = 0;
        curr_x = 0;
        curr_y = 0;
        fdbk_spd = 0.0;
        fdbk_yaw_org = 0.0;
        fdbk_yaw = 0.0;
        trgt_spd = 0;
        accel = 0;
        steering_angle = 0;
        end_of_path = [0 0];
        
        %Message information
        curr_lane=1;
        message_channel;
        curr_rd_sgmnt =[];
        nxt_rd_sgmnt = [];
        curr_vertex = -1;
        nxt_vertex = -1;
        arrival_time=0;
        exit_time=0;
        trajectory_list=[];
        arrivial_time_list=[];
        sequence_num=1;
        disable_active_logging = false;
        
        intersection_status = Rgn_of_Intrst_Enum.NoIntersection;
        incoming_message_queue;
        outgoing_message_queue;
        %PID
        accelPID;
        accelSlew;
        steeringPID;
        steeringSaturate;
        forwardSaturate;
        steeringSlew;
        pathComplete =false;    
    end
    
    methods(Access  = public)
        
        %% CONSTRUCTOR
        function obj = Autonomous_Vehicle(model_name, vehicle_id, message_channel, map, speed_limit, turn_limit, path, pos , L_axle, yaw)
            %Inports from Java
            import java.util.LinkedList
            
            obj.start_time = 0;%get_param(model_name,'SimulationTime');
            obj.trgt_spd=obj.speed_limit;
            
            
            obj.curr_x_rear = pos(1);
            obj.curr_y_rear = pos(2);
            obj.L_axle = L_axle;
            obj.fdbk_yaw_org = yaw;
            obj.fdbk_yaw = wrapTo360(yaw);
            obj.feedback_path = [];
            
            if 0 <= yaw && yaw < 90
                obj.vehicle_curr_trgt_dir = Dir_Enum.Nrth;
                obj.vehicle_trgt_dir = Dir_Enum.Nrth;
            elseif 90 <= yaw && yaw < 180
                obj.vehicle_curr_trgt_dir = Dir_Enum.Est;
                obj.vehicle_trgt_dir = Dir_Enum.Est;
            elseif 180 <= yaw && yaw < 270
                obj.vehicle_curr_trgt_dir = Dir_Enum.Sth;
                obj.vehicle_trgt_dir = Dir_Enum.Sth;
                obj.use_est_pos  = true;
            else
                obj.vehicle_curr_trgt_dir = Dir_Enum.Wst;
                obj.vehicle_trgt_dir = Dir_Enum.Wst;
                obj.use_est_pos  = true;
            end
            
            [obj.curr_x,obj.curr_y] = obj.calcFrontAxle( ...
                 obj.curr_x_rear,obj.curr_y_rear, L_axle,yaw);
            %Constants
            obj.model_name = model_name;
            obj.speed_limit= speed_limit;
            obj.turn_limit = turn_limit;
            obj.vehicle_id = vehicle_id;
            
            %Filepath names
            obj.feedback_pos_file_names = ['SavedData/test_1_vehicle_' num2str(vehicle_id) ...
                '_feedback_pos.mat'];
            obj.estimated_pos_file_names = ['SavedData/test_1_vehicle_' num2str(vehicle_id) ...
                '_estimated_pos.mat'];
            
            if exist(obj.feedback_pos_file_names, 'file')==2
                delete(obj.feedback_pos_file_names);
                delete(obj.estimated_pos_file_names);
            end 
            %strrep(strrep(datestr(datetime('now')), ' ', '-'),':','-')
            
            %PIDs
            obj.accelPID = PID(0.85, 0,0);
            obj.accelSlew = SlewRate(4,4,0);
            
            obj.steeringPID = PID(0.77, 0, 0); %0.75, 0.02
            obj.steeringSlew = SlewRate(60,60,0);            
            obj.steeringSaturate = Saturate(-170,170);
            obj.forwardSaturate = Saturate(-10,10);

            %Maps/path
            obj.map = map;
            if 1 < length(path) 
                [obj.path, ~]= map.shrtest_pth(path(1),path(2));
            else
                from = map.get_closest_node(pos(1),pos(2));
                [obj.path, ~]= map.shrtest_pth(from,path);
            end
            obj.message_channel = message_channel;
            
            %State of Car
            obj.incoming_message_queue = LinkedList();
            obj.outgoing_message_queue = LinkedList();
            obj.next_node();
            
            
        end
        
        %% STATE UPDATING
        
        function updateState(obj,  front_pos, rear_pos, fdbk_spd, fdbk_yaw)
            %try
            %debug_command = (['[target_speed,accel, streering_angle] = vvv.update(['...
            %    int2str([new_x, new_y]) '] , [' int2str(fdbk_spd) '])'])
            
            obj.curr_x = front_pos(1);
            obj.curr_y = front_pos(2);
            
            obj.curr_x_rear = rear_pos(1);
            obj.curr_y_rear = rear_pos(2);
            
            new_path_to_add = [ obj.curr_y; obj.curr_x];
            obj.feedback_path = [obj.feedback_path new_path_to_add];
            
            if obj.feedback_pos_write_buffer < length(obj.feedback_path)
                append_to_path(obj.feedback_pos_file_names, ...
                    obj.feedback_path);
                obj.feedback_path = [];
                
            end
            %disp(['TEST Current Rear X:' num2str(obj.curr_x_rear) newline]);
            %disp(['TEST Current Rear Y:' num2str(obj.curr_y_rear) newline]);
            %disp(['TEST Current Front X:' num2str(obj.curr_x) newline]);
            %disp(['TEST Current Front Y:' num2str(obj.curr_y) newline newline newline]);
            

            obj.fdbk_spd = fdbk_spd;
            obj.fdbk_yaw_org = fdbk_yaw;
            fdbk_yaw = wrapTo360(fdbk_yaw);
            obj.fdbk_yaw = fdbk_yaw;
            
            if length(obj.path) <=1
                if ~obj.pathComplete
                   obj.accelPID = PID(0.85, 0.09,0);
                   obj.pathComplete = true;
                end
                obj.trgt_spd =0;
                obj.calcAccel(fdbk_spd);
                obj.steering_angle = 0;
                return
            end
            
            %Calculations of the new state of the vehicle
            obj.calc_exit_time();
            
            %Checking for a node transition
            if obj.vehicle_plan ~= Vehicle_Movement.Turning_Right
                dist_to_vertex = obj.get_dist_to_nxt_vertex(obj.curr_x, obj.curr_y);
            else
                %dist_to_vertex = obj.get_dist_to_nxt_vertex(obj.curr_x_rear, obj.curr_y_rear);
                dist_to_vertex = obj.get_dist_to_nxt_vertex(obj.curr_x, obj.curr_y);
            end
            
            %dist_to_vertex = sqrt( (obj.end_of_path(1) -  obj.curr_x)^2 + ...
            %    (obj.end_of_path(2)- obj.curr_y)^2);
            
            
            if dist_to_vertex <=  obj.min_thresh_to_nxt_vertex
                obj.next_node();
            end
            
            %Process any messages
            obj.processMessages();
            
            %Send any messages
            obj.sendMessage();
            
            [x_next,y_next] = obj.updateEstPath();
            if 1 <= length(obj.path)
                obj.calcTargetSpeed(obj.curr_x,obj.curr_y,x_next,y_next);
                obj.calcAccel(fdbk_spd);
                obj.calcSteeringAngle(obj.fdbk_yaw, obj.curr_x,obj.curr_y,x_next,y_next);
            else
                obj.trgt_spd =0;
                obj.calcAccel(fdbk_spd);
                obj.steering_angle = 0;
            end
            %catch e
            %    disp('f');
            %end
        end
        
        
        
        %% SETTERS
        function recieve_message(obj, message)
            obj.incoming_message_queue.add(message);
        end
        
        %% GETTERS
        
        function queue = get_vehicle_id(obj)
            queue = obj.vehicle_id;
        end
        
        function trgt_spd = get_trgt_spd(obj)
            trgt_spd = obj.trgt_spd;
        end
        
        function accel = get_accel(obj)
            accel = obj.accel;
        end
        
        function accel = get_steering_angle(obj)
            accel = obj.steering_angle;
        end
    end
    
    methods(Access = protected)
        
        
        function next_node(obj)
            
            prev_vertex = obj.nxt_vertex;
            
            if prev_vertex == -1
                obj.intersection_status = Rgn_of_Intrst_Enum.NoIntersection;
            else
                obj.path(1) = [];
                obj.intersection_status = obj.map.get_rgn_type(obj.path(1));
            end
            
            
            %Path Planning
            if 3 < length(obj.path)
                obj.curr_rd_sgmnt  = [obj.path(1) obj.path(2)];
                obj.nxt_rd_sgmnt = [obj.path(2) obj.path(3)];
                obj.nxt_vertex = obj.path(2);
            elseif 2 < length(obj.path)
                obj.curr_rd_sgmnt  = [obj.path(1) obj.path(2)];
                obj.nxt_rd_sgmnt  = [];
                obj.nxt_vertex = obj.path(2);
            elseif 1 < length(obj.path)
                obj.curr_rd_sgmnt = [];
                obj.nxt_rd_sgmnt  = [];
                obj.nxt_vertex = obj.path(2);
            elseif 1 == length(obj.path)
                obj.nxt_vertex = obj.path(1);
                return
            else
                obj.nxt_vertex = -1;
                return
            end
            disp_msg = (['Vehicle ' num2str(obj.vehicle_id) ' New Node... ' newline num2str(obj.nxt_vertex)]);
            if  ~obj.disable_active_logging
                disp_msg              
            end
            
            if (obj.intersection_status == Rgn_of_Intrst_Enum.At || ...
                    obj.intersection_status == Rgn_of_Intrst_Enum.Exiting ||...
                    obj.intersection_status == Rgn_of_Intrst_Enum.NoIntersection )
                
                [obj.estimated_path, obj.vehicle_plan, obj.vehicle_curr_trgt_dir,...
                    obj.vehicle_trgt_dir] = ...
                    obj.map.getNextMovementPath(...
                    obj.path, obj.curr_x, obj.curr_y);
                

                for i = 1:1:(obj.estimated_path.points - 1)
                    
                    [x_rng, y_rng] = pointsOnClothoid(obj.estimated_path.x(i), obj.estimated_path.y(i), ...
                        obj.estimated_path.theta(i), obj.estimated_path.k(i), ...
                        obj.estimated_path.dk(i), obj.estimated_path.L(i), ...
                        (obj.est_pos_write_buffer/(obj.estimated_path.points - 1)));
                     
                     append_to_path(obj.estimated_pos_file_names,[y_rng; x_rng]);
                end
                              
                obj.end_of_path = [obj.estimated_path.x(end), obj.estimated_path.y(end)];
                
                
            end
            
            
            obj.arrival_time = get_param(obj.model_name,'SimulationTime');
            obj.calc_exit_time();
            
            obj.calc_trajectory_list();
            obj.calc_arrivial_time_list();
            
            
            obj.create_outgoing_status_message();
        end
        
        
        function sendMessage(obj)
            if (obj.outgoing_message_queue.size() == 0) || ~obj.message_channel.isControlCahnnelFree()
                return
            end
            message = obj.outgoing_message_queue.remove();
            obj.message_channel.sendMessage(obj.vehicle_id, message, ...
                obj.message_channel.getControlChannel());
        end
        
        
        
        function processMessages(obj)
            while  0 < obj.incoming_message_queue.size()
                message_str = obj.incoming_message_queue.remove();
                message = jsondecode(message_str);
                
                if ~isempty(message)
                    
                    disp_msg = ['Vehicle ' num2str(obj.vehicle_id) ' Received ...' newline ...
                        message_str];
                    if  ~obj.disable_active_logging
                        disp_msg
                    end
                end
                %                 switch obj.intersection_status
                %                     case Rgn_of_Intrst_Enum.NoIntersection
                %                         print 'why are you telling me about this?'
                %                     case Rgn_of_Intrst_Enum.Arriving
                %                         print 'Arriving at an Intersection'
                %
                %                         %
                %                         responce = '';
                %
                %                     case Rgn_of_Intrst_Enum.At
                %                         print 'Arrived at an Intersection'
                %
                %                         %
                %                         responce = '';
                %
                %                     case Rgn_of_Intrst_Enum.Within
                %                         print 'In the Intersection'
                %                         message = jsonencode(packWithinMessage(obj));
                %
                %                         %
                %                         responce = '';
                %
                %                     case Rgn_of_Intrst_Enum.Exiting
                %                         print 'Exiting the Intersection'
                %                         message = jsonencode(packExitingMessage(obj));
                %
                %                         %
                %                         responce = '';
                %
                %                     otherwise
                %                         error('Undefined Intersection State')
                %                 end
            end
        end
        
        function create_outgoing_status_message(obj)
            message = '';
            switch obj.intersection_status
                case Rgn_of_Intrst_Enum.NoIntersection
                    % 'Not at an intersection, no message'
                case Rgn_of_Intrst_Enum.Arriving
                    % 'Arriving at an Intersection'
                    
                case Rgn_of_Intrst_Enum.At
                    % 'Arrived at an Intersection'
                    message = jsonencode(packEnterMessage(obj));
                case Rgn_of_Intrst_Enum.Within
                    % 'In the Intersection'
                    message = jsonencode(packWithinMessage(obj));
                case Rgn_of_Intrst_Enum.Exiting
                    % 'Exiting the Intersection'
                    message = jsonencode(packExitingMessage(obj));
                otherwise
                    error('Undefined Intersection State')
            end
            
            if ~isempty(message)
                disp_msg = (['Vehicle ' num2str(obj.vehicle_id) ' Sending ...' newline ...
                    message]);
                if  ~obj.disable_active_logging
                    disp_msg              
                end
                
                obj.outgoing_message_queue.add(message);
                
                %Send Message
                obj.sequence_num = obj.sequence_num +1;
                
            end
        end
        
        function message = packEnterMessage(obj)
            % ENTER format:
            % 1. message type: ENTER
            % 2. Message Sequence Number
            % 3. vehicle ID
            % 4. Current Road Segment
            % 5. Current Lane
            % 6. Next Road Segment
            % 7. Next Vertex
            % 8. Arrival Time
            % 9. Exit Time
            % 10. Trajectory Cells List
            % 11. Cell Arrivial Time List
            
            
            message = struct;
            message.type = int32(Message_Types.Enter);
            message.seqNum = obj.sequence_num;
            message.vehicle_id = obj.vehicle_id;
            message.curr_rd_sgmnt = obj.curr_rd_sgmnt;
            message.curr_lane = obj.curr_lane;
            message.nxt_rd_sgmnt = obj.nxt_rd_sgmnt;
            message.nxt_vertex	= obj.nxt_vertex;
            message.arrival_time = round(obj.arrival_time,4);
            message.exit_time = round(obj.exit_time,4);
            message.trajectory_list = obj.trajectory_list;
            message.arrivial_time_list = round(obj.arrivial_time_list,4);
            
            obj.sequence_num = obj.sequence_num+1;
        end
        
        function message = packWithinMessage(obj)
            % Within Intersection format:
            % 1. message type: WITHIN
            % 2. Message Sequence Number
            % 3. vehicle ID
            % 4. Current Road Segment
            % 5. Current Lane
            % 6. Next Road Segment
            % 7. Next Vertex
            % 8. Arrival Time
            % 9. Exit Time
            % 10. Trajectory Cells List
            % 11. Cell Arrivial Time List
            
            message = struct;
            message.type = int32(Message_Types.Within);
            message.seqNum = obj.sequence_num;
            message.vehicle_id = obj.vehicle_id;
            message.curr_rd_sgmnt = obj.curr_rd_sgmnt;
            message.curr_lane = obj.curr_lane;
            message.nxt_rd_sgmnt = obj.nxt_rd_sgmnt;
            message.nxt_vertex	= obj.nxt_vertex;
            message.arrival_time = round(obj.arrival_time,4);
            message.exit_time = round(obj.exit_time,4);
            message.trajectory_list = obj.trajectory_list;
            message.arrivial_time_list = round(obj.arrivial_time_list,4);
            obj.sequence_num = obj.sequence_num+1;
        end
        
        function message = packExitingMessage(obj)
            % EXIT format:
            % 1. message type: EXIT
            % 2. Message Sequence Number
            % 3. vehicle ID
            
            message = struct;
            message.type = int32(Message_Types.Exiting);
            message.seqNum = obj.sequence_num;
            message.vehicle_id = obj.vehicle_id;
            obj.sequence_num = obj.sequence_num+1;
        end
        
        
        
        function dist = get_dist_to_nxt_vertex(obj,x,y)
            dist =  obj.map.get_dist_to_roi(obj.nxt_vertex, ...
                obj.vehicle_curr_trgt_dir, x, y); %vehicle_trgt_dir
        end
        
        
        function calc_exit_time(obj)
            obj.exit_time = obj.arrival_time + (...
                obj.get_dist_to_nxt_vertex(obj.curr_x_rear,obj.curr_y_rear)/abs(obj.fdbk_spd));
        end
        
        
        function calc_trajectory_list(obj)
            list =[];
            k =1;
            exit_found = false;
            while k < length(obj.path) && ~exit_found
                list = [list obj.path(k)];
                
                exit_found = (obj.map.get_rgn_type(obj.path(k)) == Rgn_of_Intrst_Enum.Exiting) &&...
                    (2 < length(list));
                
                k = k+1;
            end
            
            obj.trajectory_list = list;
        end
        
        function calc_arrivial_time_list(obj)
            times =zeros(length(obj.trajectory_list)-1,1);
            times(1) = obj.arrival_time;
            for i = 2:1:(length(times))
                [~, dist] = obj.map.shrtest_pth(obj.path(i),obj.path(i+1));
                %TODO: Remove the current distance from the next vertex
                times(i) = times(i-1) + dist/abs(obj.fdbk_spd);
            end
            obj.arrivial_time_list = times;
        end
        
        
        function calcAccel(obj, fdbk)
            
            [obj.accelPID,pid_accel] = obj.accelPID.perform_PID(obj.trgt_spd, fdbk);
            
            [obj.accelSlew, obj.accel] = obj.accelSlew.slew(pid_accel);
        end
        
        function [x_next,y_next] = updateEstPath(obj)
            tol = 10e-3;
            [x_min, y_min, piece] = get_closest_point_on_spline( ...
                obj.estimated_path, obj.curr_x, obj.curr_y, tol);
            
            [piece, pos, x_rng, y_rng] = find_position_in_spline( ...
                obj.estimated_path, x_min, y_min, 10e-3);

            try
                if obj.vehicle_plan == Vehicle_Movement.Driving_Straight
                    x_next = x_rng(pos+20);
                    y_next = y_rng(pos+20);
                elseif obj.vehicle_plan == Vehicle_Movement.Turning_Left
                    x_next = x_rng(pos+10);
                    y_next = y_rng(pos+10);
                else
                    x_next = x_rng(pos+5);
                    y_next = y_rng(pos+5);
                end
            
            %x1 = x_rng(pos+3);
            %y1 = y_rng(pos+3);
            catch e
                x_next = x_rng(end);
                y_next = y_rng(end);
                %x1 = x_rng(pos+1);
                %y1 = y_rng(pos+1);
                %error('I fail here');
            end
        end
        

        function calcSteeringAngle(obj, fdbk_yaw, curr_x, curr_y, x_next, y_next)
            
            vid = obj.vehicle_id;
            
            trgt_angle = rad2deg(atan2((x_next - curr_x),(y_next - curr_y)));
            trgt_angle = wrapTo360(trgt_angle);
            
            if 180 < (trgt_angle - fdbk_yaw)
                error = (trgt_angle- fdbk_yaw -360) ;
            elseif (trgt_angle - fdbk_yaw) < -180
                error = (trgt_angle - (fdbk_yaw-360));
            else
                error = trgt_angle - fdbk_yaw;
            end
            
            [obj.steeringPID, pid_out_ang] = obj.steeringPID.perform_PID_wth_error(error);
            
            sat_ang = obj.steeringSaturate.limit(pid_out_ang);
          
            
            [obj.steeringSlew, final_ang] =  obj.steeringSlew.slew(sat_ang);
            steering_angle = deg2rad(final_ang);

            obj.steering_angle = steering_angle;
        end

        
        
        function calcTargetSpeed(obj, curr_x,curr_y,x_next,y_next)
             %*sign(x_next-curr_x);
            if obj.intersection_status == Rgn_of_Intrst_Enum.NoIntersection || ...
                    obj.intersection_status == Rgn_of_Intrst_Enum.Exiting || ...
                    obj.vehicle_plan == Vehicle_Movement.Driving_Straight
                spd = obj.speed_limit;
            else
                spd = obj.turn_spd;
            end
            
            
            %TODO Degrade speed based on messages

            %set speed
            obj.trgt_spd= spd;
        end
        
        function [x_front,y_front] = calcFrontAxle(obj, x,y, L_axle,yaw)
            x_front = x + L_axle*cos(deg2rad(yaw - 90));
            y_front = y + L_axle*sin(deg2rad(yaw - 90));
        end  
        
        
        function [x_front,y_front] = calcRearAxle(obj, x,y, L_axle,yaw)
            x_front = x - L_axle*cos(deg2rad(yaw - 90));
            y_front = y - L_axle*sin(deg2rad(yaw - 90));
        end
    end
    
    
   
end