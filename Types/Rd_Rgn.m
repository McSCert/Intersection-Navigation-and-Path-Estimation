classdef Rd_Rgn
    %NODE_INFO Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access=private)
        spline;
        node;
        x1;
        y1;
        x2;
        y2;
        width;
        dir;
        type;
        tol;
    end
    
    methods

        %% Create intersection region of interest object
        % OBJ = RD_RGN(TYPE, F, X1,Y1, X2, Y2, WIDTH, DIR, TOL)  constructor a region of
        % interest object based on the bounding areas for a road 
        %
        function obj = Rd_Rgn(type, f, x1,y1, x2, y2, width, dir, tol)
            obj.x1 = double(x1);
            obj.y1 = double(y1);
            obj.x2 = double(x2);
            obj.y2 = double(y2);
            obj.spline = f;
            obj.width = width;
            obj.type = type;
            obj.dir = dir;
            obj.tol = tol;
        end

        %% Get the first xy point
        % [X,Y] = GET_RNG_XY1(OBJ) get the first bounding point of the region of interest
        % 
         function [x,y] = get_rng_xy1(obj)
            slope = estimate_derivative_clothoid(obj.spline, obj.x1, obj.y1, obj.tol);
            theta = atan(slope);
            x = obj.x1 + cos(theta + obj.get_x_offset())*(obj.width);
            y = obj.y1 + sin(theta + obj.get_y_offset())*(obj.width);
        end

        %% Get the second xy point
        % [X,Y] = GET_RNG_XY2(OBJ) get the second bounding point of the region of interest
        % 
        function [x,y] = get_rng_xy2(obj)
            slope = estimate_derivative_clothoid(obj.spline, obj.x2, obj.y2, obj.tol);
            theta = atan(slope);
            x = obj.x2 + cos(theta + obj.get_x_offset())*(obj.width);
            y = obj.y2 + sin(theta  + obj.get_y_offset())*(obj.width);
        end
        

        %% Get the first midpoint xy point
        % [X,Y] = GET_RD_XY1(OBJ) get the first midpoint point of the region of interest
        %
        function [x,y] = get_rd_xy1(obj)
            slope = estimate_derivative_clothoid(obj.spline, obj.x1, obj.y1, obj.tol);
            theta = atan(slope);
            x = obj.x1 + cos(theta + obj.get_x_offset())*(obj.width/2);
            y = obj.y1 + sin(theta + obj.get_y_offset())*(obj.width/2);
        end

        %% Get the second midpoint xy point
        % [X,Y] = GET_RD_XY2(OBJ) get the second midpoint point of the region of interest
        %
        function [x,y] = get_rd_xy2(obj)
            slope = estimate_derivative_clothoid(obj.spline, obj.x2, obj.y2, obj.tol);
            theta = atan(slope);
            x = obj.x2 + cos(theta + obj.get_x_offset())*(obj.width/2);
            y = obj.y2 + sin(theta  + obj.get_y_offset())*(obj.width/2);
        end


        %% Get the mimium x point
        % [X,Y] = GET_EQ_X_MIN(OBJ) get the minimum x,y point based on the x value
        %
        function [x,y] = get_eq_x_min(obj)
            if obj.x1 < obj.x2
                x = obj.x1;
                y = obj.y1;
            else
                x = obj.x2;
                y = obj.y2;
            end
            
        end
        
        %% Get the max x point
        % [X,Y] = GET_EQ_X_MAX(OBJ) get the max x,y point based on the x value
        %
         function [x,y] = get_eq_x_max(obj)
            if obj.x1 < obj.x2
                x = obj.x2;
                y = obj.y2;
            else
                x = obj.x1;
                y = obj.y1;
            end
            
        end
        
        %% Get the min y point
        % [X,Y] = GET_EQ_Y_MIN(OBJ) get the min x,y point based on the y value
        %
        function [x,y] = get_eq_y_min(obj)
            if obj.y1 < obj.y2
                x = obj.x1;
                y = obj.y1;
            else
                x = obj.x2;
                y = obj.y2;
            end
            
        end
        
        %% Get the max y point
        % [X,Y] = GET_EQ_Y_MAX(OBJ) get the max x,y point based on the y value
        %
         function [x,y] = get_eq_y_max(obj)
            if obj.y1 < obj.y2
                x = obj.x2;
                y = obj.y2;
            else
                x = obj.x1;
                y = obj.y1;
            end
            
        end

        %% Get the region of interest type
        % TYPE = GET_TYPE(OBJ) provides the region of interest type
        %
        function type = get_type(obj)
            type = obj.type;
        end
        

        %% Get the distance of the vehicle to the region of interest 
        % DIST = GET_DIST_TO_RGN(OBJ, X, Y) provides the distance to the region
        % of interest based on the current position (x,y) of the vehicle. The 
        % distance to the region of interest is based on the arclength on the target
        % road from the vehicle to the roi
        function dist = get_dist_to_rgn(obj, x, y)
            [x_c, y_c, ~] = get_closest_point_on_spline(obj.spline, x, y, obj.tol);
            %x_closest = x_c
            %y_closest = y_c
             slope = estimate_derivative_clothoid(obj.spline,  obj.x1, obj.y1, obj.tol);
             theta = atan(slope);
             dist1 =  arc_length_of_spline_section(obj.spline, obj.x1, obj.y1, ...
               x_c, y_c, obj.tol);
            dist2 = arc_length_of_spline_section(obj.spline, obj.x2, obj.y2, ...
               x_c, y_c, obj.tol);
            dist = min(dist1, dist2);
        end
        

        
        %% Get the closest x, y point 
        % [X_MIN,Y_MIN] = GET_CLOSEST_XY(OBJ, X, Y)
        % For a given X, Y, provides the closest points to the region of
        % interest spline (X_MIN,Y_MIN)
        function [x_min,y_min] = get_closest_xy(obj, x, y)
           [x_c, y_c, ~] = get_closest_point_on_spline(obj.spline, x, y, obj.tol);
           
           dist1 =  arc_length_of_spline_section(obj.spline, obj.x1, obj.y1, ...
               x_c, y_c, obj.tol);
           dist2 = arc_length_of_spline_section(obj.spline, obj.x2, obj.y2, ...
               x_c, y_c, obj.tol);
           if dist1 <dist2
                x_min = obj.x1;
                y_min = obj.y1;
           else
                x_min = obj.x2;
                y_min = obj.y2;
           end
        end
        
        %% Get the furthest x, y point 
        % [X_MIN,Y_MIN] = GET_FURTHER_XY(OBJ, X, Y)
        % For a given X, Y, provides the furthest points to the region of
        % interest's spline (X_MIN,Y_MIN)
        function [x_min,y_min] = get_further_xy(obj, x, y)
           [x_c, y_c, ~] = get_closest_point_on_spline(obj.spline, x, y, obj.tol);
           dist1 =  arc_length_of_spline_section(obj.spline, obj.x1, obj.y1, ...
               x_c, y_c, obj.tol);
           dist2 = arc_length_of_spline_section(obj.spline, obj.x2, obj.y2, ...
               x_c, y_c, obj.tol);
           if dist2 <= dist1
                x_min = obj.x1;
                y_min = obj.y1;
           else
                x_min = obj.x2;
                y_min = obj.y2;
           end
        end
        
        %% Get the distance to x1, y1  
        % DIST = GET_DIST_TO_XY1(OBJ, X, Y)
        % For a given X, Y, provides the distance (DIST) to the first x, y 
        % pair for the region of interest's spline
        function dist = get_dist_to_xy1(obj, x, y)
            [x_c, y_c, ~] = get_closest_point_on_spline(obj.spline, x, y, obj.tol);
            dist =  arc_length_of_spline_section(obj.spline, obj.x1, obj.y1, ...
               x_c, y_c, obj.tol);
        end
        
        %% Get the distance to x2, y2  
        % DIST = GET_DIST_TO_XY2(OBJ, X, Y)
        % For a given X, Y, provides the distance (DIST) to the second x, y 
        % pair for the region of interest's spline
        function dist = get_dist_to_xy2(obj, x, y)
            [x_c, y_c, ~] = get_closest_point_on_spline(obj.spline, x, y, obj.tol);
            dist = arc_length_of_spline_section(obj.spline, obj.x2, obj.y2, ...
               x_c, y_c, obj.tol);
        end
        
        %% Get the midpoint of the region 
        % [X_AVG, Y_AVG] = GET_REGION_MIDPOINT(OBJ)
        % Calculates the region of interest's spline midpoint (X_AVG, Y_AVG)
        function [x_avg, y_avg] = get_region_midpoint(obj)
            [rd_x1,rd_y1] = get_rd_xy1(obj);
            [rd_x2,rd_y2] = get_rd_xy2(obj);
            x_avg = (rd_x1 + rd_x2)/2;
            y_avg = (rd_y1 + rd_y2)/2;
        end
        

        %% Get the closest x, y point 
        % [X_MIN,Y_MIN] = GET_CLOSEST_RD_XY(OBJ, X, Y)
        % For a given X, Y, provides the closest points to the region of
        % interest based on the midpoint of the road (X_MIN,Y_MIN)
        function [x_min,y_min] = get_closest_rd_xy(obj, x, y)
           [rd_x1,rd_y1] = get_rd_xy1(obj);
           [rd_x2,rd_y2] = get_rd_xy2(obj);
           [x_c, y_c, ~] = get_closest_point_on_spline(obj.spline, x, y, obj.tol);
           
           dist1 =  arc_length_of_spline_section(obj.spline, obj.x1, obj.y1, ...
               x_c, y_c, obj.tol);
           dist2 = arc_length_of_spline_section(obj.spline, obj.x2, obj.y2, ...
               x_c, y_c, obj.tol);
           if dist1 <dist2
                x_min = rd_x1;
                y_min = rd_y1;
           else
                x_min = rd_x2;
                y_min = rd_y2;
           end
        end
        
        %% Get the furthest x, y point 
        % [X_MIN,Y_MIN] = GET_FURTHER_RD_XY(OBJ, X, Y)
        % For a given X, Y, provides the furthest points to the region of
        % interest based on the midpoint of the road (X_MIN,Y_MIN)
        function [x_min,y_min] = get_further_rd_xy(obj, x, y)
           [rd_x1,rd_y1] = get_rd_xy1(obj);
           [rd_x2,rd_y2] = get_rd_xy2(obj);
           [x_c, y_c, ~] = get_closest_point_on_spline(obj.spline, x, y, obj.tol);
           dist1 =  arc_length_of_spline_section(obj.spline, obj.x1, obj.y1, ...
               x_c, y_c, obj.tol);
           dist2 = arc_length_of_spline_section(obj.spline, obj.x2, obj.y2, ...
               x_c, y_c, obj.tol);
           if dist2 <= dist1
                x_min = rd_x1;
                y_min = rd_y1;
           else
                x_min = rd_x2;
                y_min = rd_y2;
           end
        end
        
        %% Get the x offset
        % OFFSET = GET_X_OFFSET(OBJ)
        % Get the offset angle for the x axis
        function offset = get_x_offset(obj)
           offset = pi/2;
        end
        
        %% Get the y offset
        % OFFSET = GET_Y_OFFSET(OBJ)
        % Get the offset angle for the y axis
        function offset = get_y_offset(obj)
          offset = pi/2;
        end
        
        %% Get the road direction
        % DIR= GET_DIR(OBJ)
        % Get the direction (DIR) of the road
        function dir= get_dir(obj)
            dir =obj.dir;
        end

        %% Get type of region of interest
        % STR = GET_TYPE_STR(OBJ)
        % Provides the type of the region of interest in 
        % a string format  (STR)
        function str = get_type_str(obj)
            str = char(obj.type);
        end
        

        %% Get type of region of interest of the road
        % STR = DISP_RD(OBJ)
        % Provides the information of the region of interest in 
        % a string format  (STR) for the road
        function str = disp_rd(obj)
            [rd_x1,rd_y1] = obj.get_rd_xy1();
            [rd_x2,rd_y2] = obj.get_rd_xy2();
            str = (['Type: '  obj.get_type_str()...
                  ', x1 ' num2str(rd_x1) ...
                  ', y1 ' num2str(rd_y1)...
                  ', x2 ' num2str(rd_x2)...
                  ', y2 ' num2str(rd_y2)]);  
        end
        
        %% Get type of region of interest 
        % STR = DISP_RGN(OBJ)
        % Provides the information of the region of interest in 
        % a string format  (STR)
        function str = disp_rgn(obj)
            [rd_mid_x1,rd_mid_y1] = obj.get_rd_xy1();
            [rd_mid_x2,rd_mid_y2] = obj.get_rd_xy2();
            [end_rgn_x1,end_rgn_y1] = obj.get_rng_xy1();
            [end_rgn_x2,end_rgn_y2] = obj.get_rng_xy2();
            str = (['Type: '  obj.get_type_str()...
                  ', x_srt_1 ' num2str(obj.x1) ...
                  ', y_srt_1 ' num2str(obj.y1)...
                  ', x_srt_2 ' num2str(obj.x2)...
                  ', y_srt_2 ' num2str(obj.y2) ...
                  ', x_mid_1 ' num2str(rd_mid_x1) ...
                  ', y_mid_1 ' num2str(rd_mid_y1)...
                  ', x_mid_2 ' num2str(rd_mid_x2)...
                  ', y_mid_2 ' num2str(rd_mid_y2)...
                  ', x_end_1 ' num2str(end_rgn_x1) ...
                  ', y_end_1 ' num2str(end_rgn_y1)...
                  ', x_end_2 ' num2str(end_rgn_x2)...
                  ', y_end_2 ' num2str(end_rgn_y2)
                  ]);  
        end
        
        %% Get all of the important points for the region of interest
        % RGN = GET_RGN(OBJ)
        % Test method - Get all of the important points (rgn)
        % for the region of interest
        function rgn = get_rgn(obj)
            [rd_mid_x1,rd_mid_y1] = obj.get_rd_xy1();
            [rd_mid_x2,rd_mid_y2] = obj.get_rd_xy2();
            [end_rgn_x1,end_rgn_y1] = obj.get_rng_xy1();
            [end_rgn_x2,end_rgn_y2] = obj.get_rng_xy2();
            
            rgn = [obj.x1 obj.y1;  obj.x2 obj.y2; ...
                  rd_mid_x1,rd_mid_y1; rd_mid_x2,rd_mid_y2; ...
                  end_rgn_x1,end_rgn_y1; end_rgn_x2,end_rgn_y2 ...
            ];
        end
        
        %% Get all of the important points for the region of interest, rounded
        % RGN = GET_RGN(OBJ)
        % Test method - Get and rounds all of the important points (rgn)
        % for the region of interest
        function str = disp_rounded_rgn(obj)
            [rd_mid_x1,rd_mid_y1] = obj.get_rd_xy1();
            [rd_mid_x2,rd_mid_y2] = obj.get_rd_xy2();
            [end_rgn_x1,end_rgn_y1] = obj.get_rng_xy1();
            [end_rgn_x2,end_rgn_y2] = obj.get_rng_xy2();
            str = (['Type: '  obj.get_type_str()...
                  ', x_srt_1 ' num2str(roundn(obj.x1,-1)) ...
                  ', y_srt_1 ' num2str(roundn(obj.y1,-1))...
                  ', x_srt_2 ' num2str(roundn(obj.x2,-1))...
                  ', y_srt_2 ' num2str(roundn(obj.y2,-1)) ...
                  ', x_mid_1 ' num2str(roundn(rd_mid_x1,-1)) ...
                  ', y_mid_1 ' num2str(roundn(rd_mid_y1,-1))...
                  ', x_mid_2 ' num2str(roundn(rd_mid_x2,-1))...
                  ', y_mid_2 ' num2str(roundn(rd_mid_y2,-1))...
                  ', x_end_1 ' num2str(roundn(end_rgn_x1,-1)) ...
                  ', y_end_1 ' num2str(roundn(end_rgn_y1,-1))...
                  ', x_end_2 ' num2str(roundn(end_rgn_x2,-1))...
                  ', y_end_2 ' num2str(roundn(end_rgn_y2,-1))
                  ]);  
        end
    end
end

