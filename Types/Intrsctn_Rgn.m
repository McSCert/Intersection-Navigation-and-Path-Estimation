classdef Intrsctn_Rgn
    %INTRSCTN_RGN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties 
        rl_rgn;
        td_rgn;
        type;
        quad;
    end
    
    methods

        %% Create intersection region of interest object
        % OBJ = INTRSCTN_RGN(TYPE, RL_RGN ,TD_RGN, QUAD)  constructor a region of
        % interest object based on the right and left bounding areas (rl_rgn) and
        %the top down bounding areas (td_rgn). For a specific quarderant (QUAD)
        %
        function obj = Intrsctn_Rgn(type, rl_rgn,td_rgn, quad)
           obj.rl_rgn = rl_rgn;
           obj.td_rgn = td_rgn;
           obj.quad = quad;
           obj.type = type;
        end
        

        %% Get the right/left road region of interest
        % RL_RGN = GET_RL_RGN(OBJ) - get the object for the region of interest
        % for the right/left road (RL_RGN)
        %
        function rl_rgn = get_rl_rgn(obj)
           rl_rgn = obj.rl_rgn();
        end
        
        %% Get the top/down road region of interest
        % TD_RGN = GET_TD_RGN(OBJ) - get the object for the region of interest
        % for the top/down road (TD_RGN)
        %
        function td_rgn = get_td_rgn(obj)
           td_rgn = obj.td_rgn();
        end
        
        %% Get the midpoint of the region of interest
        % [X_AVG, Y_AVG] = GET_REGION_MIDPOINT(OBJ) - get the midpoint (X_AVG,
        % Y_AVG) for the region of interest
        %
        function [x_avg, y_avg] = get_region_midpoint(obj)
            [x_avg_1,y_avg_1]  = obj.rl_rgn().get_region_midpoint();
            [x_avg_2,y_avg_2]  = obj.td_rgn().get_region_midpoint();
            
            %Quad == 1 - Top Right
            %Quad == 2 - Top Left
            %Quad == 3 - Bot Right
            %Quad == 4 - Bot Left
            if obj.quad == 2 || obj.quad == 4
                x_avg = max(x_avg_1, x_avg_2);
            else
                x_avg = min(x_avg_1, x_avg_2);
            end
            
            if obj.quad == 3 || obj.quad == 4
                y_avg = min(y_avg_1, y_avg_2);
            else
                y_avg = max(y_avg_1, y_avg_2);
            end
        end
        
        
        %% Get the midpoint of the region of interest
        % TYPE = GET_TYPE(OBJ) - Return the TYPE of the region of interest
        %
        function type = get_type(obj)
            type = obj.type;
        end
        
        %% Get the midpoint of the region of interest
        % STR = GET_TYPE_STR(OBJ) - Return the TYPE of the region of interest
        % in the form of a  string (STR)
        %
        function str = get_type_str(obj)
            str = char(obj.type);
        end
        
        
        %% Get the north bounding poinnt of the region of interest
        % [X, Y] = GET_FURTHEST_NORTHERN_POINT(OBJ) - Return the north bounding 
        % point (X, Y) of the region of interest
        %
        function [x, y] = get_furthest_northern_point(obj)
            [x1,~] = obj.rl_rgn().get_rd_xy1();
            [x2,~] = obj.rl_rgn().get_rd_xy2();
            [~,y1] = obj.td_rgn().get_rd_xy1();
            [~,y2] = obj.td_rgn().get_rd_xy2();
            
            x = mean([x1 x2]);
            y= max([y1,y2]);
        end
        
        %% Get the south bounding poinnt of the region of interest
        % [X, Y] = GET_FURTHEST_SOUTHERN_POINT(OBJ) - Return the south bounding 
        % point (X, Y) of the region of interest
        %
        function [x, y] = get_furthest_southern_point(obj)
            [x1,~] = obj.rl_rgn().get_rd_xy1();
            [x2,~] = obj.rl_rgn().get_rd_xy2();
            [~,y1] = obj.td_rgn().get_rd_xy1();
            [~,y2] = obj.td_rgn().get_rd_xy2();
            
            x = mean([x1 x2]);
            y= min([y1,y2]);
        end
        
        %% Get the east bounding poinnt of the region of interest
        % [X, Y] = GET_FURTHEST_EASTERN_POINT(OBJ) - Return the east bounding 
        % point (X, Y) of the region of interest
        %
        function [x, y] = get_furthest_eastern_point(obj)
            [x1,~] = obj.rl_rgn().get_rd_xy1();
            [x2,~] = obj.rl_rgn().get_rd_xy2();
            [~,y1] = obj.td_rgn().get_rd_xy1();
            [~,y2] = obj.td_rgn().get_rd_xy2();
            
            x = max([x1 x2]);
            y= mean([y1,y2]);
        end
        
        %% Get the west bounding poinnt of the region of interest
        % [X, Y] = GET_FURTHEST_WESTERN_POINT(OBJ) - Return the west bounding 
        % point (X, Y) of the region of interest
        %
        function [x, y] = get_furthest_western_point(obj)
             [x1,~] = obj.rl_rgn().get_rd_xy1();
             [x2,~] = obj.rl_rgn().get_rd_xy2();
             [~,y1] = obj.td_rgn().get_rd_xy1();
             [~,y2] = obj.td_rgn().get_rd_xy2();
            
            x = min([x1 x2]);
            y= mean([y1,y2]);
        end
    end
end

