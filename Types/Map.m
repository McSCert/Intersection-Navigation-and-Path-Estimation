classdef Map
    %MAP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access = private)
        graph
        intrsct_mp
        intrsct_idx_mp %[sorted numbers]
        rgns_of_intrst
        equations
        intrsct_pos_mp
    end
    
    methods
        
        %% Create Map Object
        % OBJ = MAP(MAT_FILE ,TOL) Creates a directed graph based on the
        % scenario given in MAT_FILE, based on the tolerances given by TOL
        function obj = Map(mat_file ,tol)
            %MAP contructs a map given a senerio of different roads within
            %an enviroment
            
            if isa(mat_file, 'struct')
                map = mat_file;
            else
                map = load(mat_file);
            end
            map = map.data;
            roads= map.RoadSpecifications;
            obj.rgns_of_intrst = {};
            [intrsct_idx_mp, intrsct_pos_mp, equations]=  obj.get_intrsct_points(roads,tol);
            
            obj.intrsct_idx_mp = intrsct_idx_mp;
            obj.intrsct_pos_mp = intrsct_pos_mp;
            obj.equations = equations;
            [obj.graph,obj.intrsct_mp, obj.rgns_of_intrst] = obj.build_graph(roads, intrsct_idx_mp, intrsct_pos_mp, equations, tol);
        end
        
    end
    methods (Access = public)
        
        %% Retrieve a path of motion for a given path
        % [EQ, TRN_TYPE, FROM_DIR, TO_DIR] = GETNEXTMOVEMENTPATH(OBJ, PATH,
        %X, Y) Provides the next movement path for a given vehicle at (X,
        %Y) and a given PATH
        %
        % OUTPUTS
        %
        % EQ - A spline for a path of motion
        % TRN_TYPE - The type of turn for the vehicle 
        % FROM_DIR - The starting direction of the vehicle
        % TO_DIR - The ending direction of the vehicle
        function [eq, trn_type, from_dir, to_dir] = getNextMovementPath(obj, path, x, y)
            first_node = path(1);
            if (obj.rgns_of_intrst{first_node}.get_type() == Rgn_of_Intrst_Enum.At)
                exit_roi = Rgn_of_Intrst_Enum.Exiting;
                
                at_intrsct_indx = 1;
                while obj.rgns_of_intrst{path(at_intrsct_indx)}.get_type() ~=exit_roi
                    at_intrsct_indx = at_intrsct_indx+1;
                end
                end_of_trn = path(at_intrsct_indx);
                
                from_dir = obj.rgns_of_intrst{first_node}.get_dir();
                to_dir = obj.rgns_of_intrst{end_of_trn}.get_dir();
                trn_type = get_turn_type(obj, from_dir, to_dir);
                
                [x_nxt,y_nxt] = obj.rgns_of_intrst{first_node}.get_closest_rd_xy(x, y);
                subset_path = [x_nxt,y_nxt];
                within_count =0;
                after_lft_turn = false;
                for i =1:1:at_intrsct_indx
                    [~, e_outgoing] = outedges(obj.graph,path(i));
                    for j = 1:1:length(e_outgoing)
                        if ~isempty(path(path==e_outgoing(j)))
                            x_nxt = [];
                            y_nxt= [];
                            type = obj.rgns_of_intrst{path(i)}.get_type();
                            if type == Rgn_of_Intrst_Enum.Within
                                within_count = within_count+1;
                                
                                if (trn_type == Vehicle_Movement.Turning_Right)
                                    [x_nxt,y_nxt] = obj.get_next_path_for_turn(obj.rgns_of_intrst{path(i)}, to_dir);
                                    
                                elseif (trn_type == Vehicle_Movement.Driving_Straight)
                                    [x_nxt,y_nxt] = obj.get_next_path_for_turn(obj.rgns_of_intrst{path(i)}, to_dir);
                                    
                                else
                                    if within_count == 1
                                        [x_nxt(1),y_nxt(1)] = obj.rgns_of_intrst{path(i)}.get_region_midpoint();
                                        [x_nxt(2),y_nxt(2)] =  obj.get_next_path_for_turn(obj.rgns_of_intrst{path(i)}, from_dir);
                                    else
                                        if ~after_lft_turn
                                            [x_nxt,y_nxt] = obj.get_next_path_for_turn(obj.rgns_of_intrst{path(i)}, to_dir);
                                            after_lft_turn = true;
                                        else
                                            [x_nxt(1),y_nxt(1)] = obj.rgns_of_intrst{path(i)}.get_region_midpoint();
                                            [x_nxt(2),y_nxt(2)] = obj.get_next_path_for_turn(obj.rgns_of_intrst{path(i)}, to_dir);
                                        end
                                    end
                                end
                                
                            elseif type == Rgn_of_Intrst_Enum.At || type == Rgn_of_Intrst_Enum.Exiting
                                x_nxt = zeros(2,1);
                                y_nxt = zeros(2,1);
                                [x_nxt(1),y_nxt(1)] = obj.rgns_of_intrst{path(i)}.get_region_midpoint();
                                [x_nxt(2),y_nxt(2)] = obj.rgns_of_intrst{path(i)}.get_further_rd_xy(x, y);
                            end
                            for k = 1:1:length(x_nxt)
                                subset_path = [subset_path; x_nxt(k),y_nxt(k)];
                            end
                        end
                    end
                end
                
            else
                from_dir = obj.rgns_of_intrst{first_node}.get_dir();
                
                at_intrsct = Rgn_of_Intrst_Enum.At;
                at_intrsct_indx = 1;
                while obj.rgns_of_intrst{path(at_intrsct_indx)}.get_type() ~=at_intrsct
                    at_intrsct_indx = at_intrsct_indx+1;
                end
                strt_of_intrsct = path(at_intrsct_indx);
                
                to_dir = obj.rgns_of_intrst{strt_of_intrsct}.get_dir();
                trn_type = Vehicle_Movement.Driving_Straight;
                [x_nxt,y_nxt] = obj.rgns_of_intrst{path(1)}.get_closest_rd_xy(x, y);
                subset_path = [x_nxt,y_nxt];
                
                %[x_nxt,y_nxt] = obj.rgns_of_intrst{path(1)}.get_further_xy(x, y);
                %subset_path = [subset_path; x_nxt,y_nxt];
                
                for i = 2:1:at_intrsct_indx
                    [x_nxt,y_nxt] = obj.rgns_of_intrst{path(i)}.get_closest_rd_xy(x, y);
                    subset_path = [subset_path; x_nxt,y_nxt];
                end
                
                [x_nxt,y_nxt] = obj.rgns_of_intrst{strt_of_intrsct}.get_further_rd_xy(x, y);
                subset_path = [subset_path; x_nxt,y_nxt];
            end
            eq.x = subset_path(:,1);
            eq.y = subset_path(:,2);
            eq.points = length(subset_path);
            
            
            [eq.theta,eq.k,eq.dk,eq.L,eq.nevalG1, ...
                eq.nevalF,eq.iter,eq.Fvalue,eq.Fgradnorm] = G1spline(subset_path);
            
            
        end
        
        function num_of_roi = get_num_of_roi(obj)
            num_of_roi = length(obj.rgns_of_intrst);
        end
        
        function roi = get_roi(obj, i)
            roi = obj.rgns_of_intrst{i};
        end
        
        function dist = get_dist_to_roi(obj, to, direction, x, y)
            %x=x
            %y=y
            if isa(obj.rgns_of_intrst{to},'Rd_Rgn')
                dist = obj.rgns_of_intrst{to}.get_dist_to_rgn(x,y);
                %dist
            else
                if  direction == Dir_Enum.Est || direction == Dir_Enum.Wst
                    dist = obj.rgns_of_intrst{to}.get_rl_rgn().get_dist_to_rgn(x,y);
                else
                    dist = obj.rgns_of_intrst{to}.get_td_rgn().get_dist_to_rgn(x,y);
                end
                
            end
        end
        
        function type = get_rgn_type(obj, num)
            if num <= length(obj.rgns_of_intrst)
                type = obj.rgns_of_intrst{num}.get_type();
            else
                type = Rgn_of_Intrst_Enum.NoIntersection;
            end
        end
        
        function [path, dist] = shrtest_pth(obj, from, to)
            [path,dist] = shortestpath(obj.graph,from,to);
        end
        
        function [node] = get_closest_node(obj, x, y)
            node = null(1);
            min_dist = Inf;
            for i = 1:1:length(obj.rgns_of_intrst)
                
                if obj.rgns_of_intrst{i}.get_type() == Rgn_of_Intrst_Enum.Within
                    try
                        new_dist = obj.rgns_of_intrst{i}.get_rl_rgn().get_dist_to_rgn(x, y);
                        
                    catch exception
                        new_dist = obj.rgns_of_intrst{i}.get_td_rgn().get_dist_to_rgn(x, y);
                    end
                else
                    new_dist = obj.rgns_of_intrst{i}.get_dist_to_rgn(x, y);
                end
                if new_dist < min_dist
                    min_dist = new_dist;
                    node = i;
                end
            end
        end
        
        
        %% Display the directed graph
        % DISP_GRAPH(OBJ) - Displays the directed graph as a plot
        function disp_graph(obj)
            p = plot(obj.graph,'EdgeLabel',obj.graph.Edges.Weight);
            grid on
            
            x_vals = zeros(length(obj.rgns_of_intrst),1);
            y_vals = zeros(length(obj.rgns_of_intrst),1);
            % Find the mid points of every node based on the region of
            % interest
            for i =1:1:length(obj.rgns_of_intrst)
                if obj.rgns_of_intrst{i}.get_type() == Rgn_of_Intrst_Enum.Within
                    [x_vals(i), y_vals(i)] = obj.rgns_of_intrst{i}.get_region_midpoint();
                else
                    [x_vals(i), y_vals(i)] = obj.rgns_of_intrst{i}.get_region_midpoint();
                end
            end
            
            x_max = max(x_vals);
            %x_min = min(x_vals);
            %y_min = min(y_vals);
            %y_max = max(y_vals);
            
            % flip the x coord to match the birds eye scope
            x_vals = x_max - x_vals;
            p.XData = x_vals;
            p.YData = y_vals;
            p.ArrowSize = 15;
            p.LineWidth = 4;
            p.MarkerSize = 6;
            p.EdgeFontSize = 20;
            
            %axis([(x_min-10) (x_max+10) (y_min-10) (y_max+10)])
            xlabel('x');
            ylabel('y');
            set(gca,'FontSize',40);
        end
        
        %% TEST METHOD - Display the directed graph during object contruction
        % TEST_DISP_GRAPH(OBJ, TOEDGES, FROMEDGES, WEIGHTS, INTRSCT_MP ,ROI) - Displays the directed graph as a plot by
        %using the values that are generated during the map object creation 
        function test_disp_graph(obj, toEdges, fromEdges, weights, intrsct_mp ,roi)
            digraph_obj = digraph(fromEdges,toEdges,weights);
            p = plot(digraph_obj,'EdgeLabel',digraph_obj.Edges.Weight);
            grid on
            
            x_vals = zeros(length(roi),1);
            y_vals = zeros(length(roi),1);
            for i =1:1:length(roi)
                if roi{i}.get_type() == Rgn_of_Intrst_Enum.Within
                    [x_vals(i), y_vals(i)] = roi{i}.get_region_midpoint();
                else
                    [x_vals(i), y_vals(i)] = roi{i}.get_region_midpoint();
                end
            end
            
            x_max = max(x_vals);
            
            x_vals = x_max - x_vals;
            p.XData = x_vals;
            p.YData = y_vals;
            
            xlabel('x');
            ylabel('y');
        end
        
        
        %% TEST METHOD - Display the directed graph without placing the nodes based on X, Y positions
        % DISP_GRAPH_ORG(OBJ) - display the directed graph without placing
        % them based on the midpoints of the regions of interest
        function disp_graph_org(obj)
            LWidths = 5*obj.graph.Edges.Weight/max(obj.graph.Edges.Weight);
            plot(obj.graph,'EdgeLabel',obj.graph.Edges.Weight,'LineWidth',LWidths)
        end

        
        %% Display the region of interest bounding points
        % DISP_ROI(OBJ) - Displays the bounding for every region of interest
        function disp_roi(obj)
            for i=1:1:length(obj.rgns_of_intrst)
                intrst_rgn = obj.rgns_of_intrst{i};
                if isa(intrst_rgn,'Rd_Rgn')
                    disp(['Node: ' num2str(i) ', ' intrst_rgn.disp_rgn()]);
                else
                    disp(['Node: ' num2str(i) ...
                        newline 'Right Rd ' intrst_rgn.get_rl_rgn().disp_rgn()...
                        newline 'Top Rd ' intrst_rgn.get_td_rgn().disp_rgn()]);
                end
                disp(newline)
            end
        end
        
        %% TEST METHOD - Display the region of interest bounding points
        % DISP_ROI(OBJ) - Displays the bounding for every region of
        % interest with the x,y points rounded to one decimal point
        function disp_rounded_roi(obj)
            for i=1:1:length(obj.rgns_of_intrst)
                intrst_rgn = obj.rgns_of_intrst{i};
                if isa(intrst_rgn,'Rd_Rgn')
                    disp(['Node: ' num2str(i) ', ' intrst_rgn.disp_rounded_rgn()]);
                else
                    disp(['Node: ' num2str(i) ...
                        newline 'Right Rd ' intrst_rgn.get_rl_rgn().disp_rounded_rgn()...
                        newline 'Top Rd ' intrst_rgn.get_td_rgn().disp_rounded_rgn()]);
                end
                disp(newline)
            end
        end
    end
    
    methods (Access = private)
        %% Get the intersection points between roads
        % [INTRSCT_IDX_MP, INTRSCT_POS_MP, EQUATIONS]=
        % GET_INTRSCT_POINTS(OBJ, ROADS,TOL) Based on the set of roads
        % ROADS, finds the intersection points of the road network using
        % the tolernace TOL
        %
        % OUTPUTS
        %
        % INTRSCT_IDX_MP - map to coorespond different intersections to
        % different roads
        % INTRSCT_POS_MP  - map to coorespond different intersections to
        % their cooresponding position
        % EQUATIONS - a set of clothoids that are used to model the roads
        
        function [intrsct_idx_mp, intrsct_pos_mp, equations]= get_intrsct_points(obj, roads,tol)
            
            equations = {};
            %syms x y
            for i= 1:1:length(roads)
                road = roads(i);
                [points, ~] = size(road.Centers);
                
                %poly = fit(road.Centers(1:points, 1),road.Centers(1:points, 2), 'smoothingspline');
                road_centers = [road.Centers(1:points, 2),road.Centers(1:points, 1), road.Centers(1:points, 3)];
                eq = estimateRoadClothoids(road_centers, ...
                    road.BankAngle*ones(points,1),points);
                
                equations{i} = eq;
            end
            
            intrsct_idx_mp = [];
            intrsct_pos_mp = [];
            % Create a matrix of the intersections for
            for i = 1:1:length(equations)
                intrsct_idxs = MatrixHelper();
                intrsct_pos = MatrixHelper();
                for j = 1:1:length(equations)
                    if i == j
                        continue
                    else
                        % ,min( roads(i).Centers(:,1)) , ...
                        %     max( roads(i).Centers(:,1)),min(roads(j).Centers(:,1)), ...
                        %     max(roads(j).Centers(:,1)))
                        [x_common, y_common] = dtrmn_intrscts_btwn_splines( ...
                            equations{i}, equations{j},tol);
                        
                        for k = 1:1:length(y_common)
                            [intrsct_idxs, intrsct_pos] =  obj.insert_intersection_point(...
                                intrsct_idxs, intrsct_pos, ...
                                x_common(k), y_common(k), i, j);
                        end
                        
                    end
                end
                
                intrsct_pos_mp = [intrsct_pos_mp intrsct_pos];
                intrsct_idx_mp = [intrsct_idx_mp intrsct_idxs];
            end
            
        end
        
        %% Insert intersection point into maps
        % [INTRSCT_IDX, INTRSCT_POS] = INSERT_INTERSECTION_POINT(OBJ,
        % INTRSCT_IDX, INTRSCT_POS, X, Y, I, J) Inserts [X, Y] into
        % INTRSCT_POS and J into INTRSCT_POS
        %
        % OUTPUTS
        %
        % INTRSCT_IDX_MP - the new map to coorespond different intersections to
        % different roads
        % INTRSCT_POS_MP  - the new map to coorespond different intersections to
        % their cooresponding position
        function [intrsct_idx, intrsct_pos] = insert_intersection_point( ...
                obj, intrsct_idx, intrsct_pos, x, y, i, j)
            %Insert an (x,y) point into the intersection list
            if intrsct_idx.isEmpty()
                intrsct_idx = intrsct_idx.addRow([j], 1);
                intrsct_pos = intrsct_pos.addRow([x y], 1);
                return
            end
            points = intrsct_idx.getSize();
            points = points(1);
            for k = 1:1:points
                x_org = intrsct_pos.getMatrixPos(k,1);
                y_org = intrsct_pos.getMatrixPos(k,2);
                
                if  coor_lessthen(x,y,x_org,y_org)
                    intrsct_idx = intrsct_idx.addRow([j], k);
                    intrsct_pos = intrsct_pos.addRow([x y], k);
                    return
                end
                
            end
            intrsct_idx = intrsct_idx.addRow([j], points+1);
            intrsct_pos = intrsct_pos.addRow([x y], points+1);
            
        end
        
        %% Build the directed graph
        % [GRAPH, INTRSCT_MP ,ROI] = BUILD_GRAPH(OBJ,ROADS, INTRSCT_IDX_MP,
        % INTRSCT_POS_MP, LINE_EQS, TOL) Builds a directed graph based on
        % the intersection points (INTRSCT_IDX_MP, INTRSCT_POS_MP) and the 
        % intersection equations with a tolerance of TOL. 
        %
        % OUTPUTS
        %
        % GRAPH - A directed graph of the road network
        % INTRSCT_MP - A mapping from (X, Y) positions to a structure that
        % contains  information about the regions of interest around an
        % intersection
        % ROI - An object array of the regions of interest within the map
        function [graph, intrsct_mp ,roi] = build_graph(obj,roads, intrsct_idx_mp, intrsct_pos_mp, line_eqs, tol)
            nodeId = 1;
            fromEdges =[];
            toEdges =[];
            weights = [];
            numOfRoads = length(roads);
            roi = {};
            intrsct_mp  = containers.Map();
            %ASSUMPTION - assuming a road width of 3m
            width =3;
            enter_dist = 2;
            hlf_wdth = width/2;
            tot_wdth = width + enter_dist;
            
            for i = 1:1:numOfRoads
                %Get the position of the intersection
                intrsct_idx = intrsct_idx_mp(i);
                intrsct_mtrx = intrsct_pos_mp(i);
                
                numOfIntsct = intrsct_idx.getSize();
                for j = 1:1:numOfIntsct(1)
                    pos = intrsct_idx.getMatrixPos(j,1);
                    if pos <= i
                        continue
                    end
                    key = [int2str(i) ',' int2str(pos)];
                    
                    if ~isKey(intrsct_mp, key) || (isKey(intrsct_mp, key) && ...
                            length(intrsct_mp(key)) < intrsct_idx.num_of_occrncs(1, pos))
                        
                        intrsct_x = intrsct_mtrx.getMatrixPos(j, 1);
                        intrsct_y = intrsct_mtrx.getMatrixPos(j, 2);
                        
                        f_i = line_eqs{i};
                        f_j = line_eqs{pos};
                        dirctn = determine_direction(f_i, f_j, intrsct_x, intrsct_y, tol);
                        
                        if dirctn == Intrsctn_Dir.EstWst
                            rght_lft =i;
                            tp_dwn = pos;
                            f_rl = line_eqs{i};
                            f_td = line_eqs{pos};
                        elseif dirctn == Intrsctn_Dir.NrthSth
                            rght_lft =pos;
                            tp_dwn = i;
                            f_rl = line_eqs{pos};
                            f_td = line_eqs{i};
                        end
                        
                        
                        
                        intrsct_strct = struct();
                        intrsct_strct.TopRight =  nodeId;
                        intrsct_strct.TopLeft =  nodeId + 1;
                        intrsct_strct.BotRight =  nodeId + 2;
                        intrsct_strct.BotLeft  =  nodeId + 3;
                        nodeId = nodeId +4;
                        
                        %In the intersection edges
                        fromEdges = [fromEdges intrsct_strct.TopRight ];
                        toEdges = [toEdges intrsct_strct.TopLeft ];
                        weights = [weights width];
                        fromEdges = [fromEdges intrsct_strct.BotLeft ];
                        toEdges = [toEdges intrsct_strct.BotRight ];
                        weights = [weights width];
                        
                        fromEdges = [fromEdges intrsct_strct.TopLeft ];
                        toEdges = [toEdges intrsct_strct.BotLeft ];
                        weights = [weights width];
                        fromEdges = [fromEdges intrsct_strct.BotRight ];
                        toEdges = [toEdges intrsct_strct.TopRight ];
                        weights = [weights width];
                        
                        
                        %Making nodes for each of the quadrents of the
                        %intersection
                        roi{intrsct_strct.TopRight} = ...
                            obj.get_intrsctn_rgn(f_rl, f_td, intrsct_x, ...
                            intrsct_y, -1, 1, 1, 1, width, width, Rd_Dir.Est, Rd_Dir.Nrth, 1, tol);
                        
                        roi{intrsct_strct.TopLeft} = ...
                            obj.get_intrsctn_rgn(f_rl, f_td, intrsct_x, ...
                            intrsct_y, 1, 1, -1, 1, width, width, Rd_Dir.Est, Rd_Dir.Sth, 2, tol);
                        
                        roi{intrsct_strct.BotRight} = ...
                            obj.get_intrsctn_rgn(f_rl, f_td, intrsct_x, ...
                            intrsct_y, -1, -1, 1, -1, width, width, Rd_Dir.Wst, Rd_Dir.Nrth, 3, tol);
                        
                        roi{intrsct_strct.BotLeft} = ...
                            obj.get_intrsctn_rgn(f_rl, f_td, intrsct_x, ...
                            intrsct_y, 1, -1, -1, -1, width, width,Rd_Dir.Wst, Rd_Dir.Sth, 4, tol);
                        
                        
                        %Getting arclengths to see what the total length
                        %going away from the interection is
                        to_intrsct_arc_length_rghtlft = arc_length_of_spline_section(line_eqs{rght_lft}, ...
                            roads(rght_lft).Centers(1,2),  roads(rght_lft).Centers(1,1), intrsct_x, intrsct_y, tol);
                        
                        frm_tnrsct_arc_length_rghtlft =  arc_length_of_spline_section(line_eqs{rght_lft}, ...
                            intrsct_x, intrsct_y, roads(rght_lft).Centers(end,2), roads(rght_lft).Centers(end,1), tol);
                        
                        to_intrsct_arc_length_tpdwn = arc_length_of_spline_section(line_eqs{tp_dwn}, ...
                            roads(tp_dwn).Centers(1,2),  roads(tp_dwn).Centers(1,1), intrsct_x, intrsct_y, tol);
                        
                        frm_tnrsct_arc_length_tpdwn =  arc_length_of_spline_section(line_eqs{tp_dwn}, ...
                            intrsct_x, intrsct_y, roads(tp_dwn).Centers(end,2), roads(tp_dwn).Centers(end,1), tol);
                        
                        %Right Side
                        if  tot_wdth < to_intrsct_arc_length_rghtlft
                            intrsct_strct.rght_wstbnd=  nodeId;
                            intrsct_strct.rght_estbnd  =  nodeId + 1;
                            
                            fromEdges = [fromEdges intrsct_strct.rght_estbnd ];
                            toEdges = [toEdges intrsct_strct.TopRight ];
                            weights = [weights    enter_dist];
                            
                            fromEdges = [fromEdges intrsct_strct.BotRight ];
                            toEdges = [toEdges intrsct_strct.rght_wstbnd ];
                            weights = [weights    enter_dist];
                            
                            [rx,ry] = roi{intrsct_strct.TopRight}.get_rl_rgn().get_eq_x_min();
                            roi{intrsct_strct.rght_wstbnd} = ...
                                obj.get_rd_rgn(f_rl, rx, ry, ...
                                -enter_dist, -width, Rgn_of_Intrst_Enum.Exiting, ...
                                Rd_Dir.Wst, tol);
                            
                            roi{intrsct_strct.rght_estbnd} = ...
                                obj.get_rd_rgn(f_rl, rx, ry, ...
                                -enter_dist, width, Rgn_of_Intrst_Enum.At, ...
                                Rd_Dir.Est, tol);
                            
                            %[trwx,trwy] = roi{intrsct_strct.rght_wstbnd}.get_region_midpoint();
                            %[trex,trey] = roi{intrsct_strct.rght_estbnd}.get_region_midpoint();
                            nodeId = nodeId +2;
                        else
                            intrsct_strct.rght_wstbnd =  zeros(0);
                            intrsct_strct.rght_estbnd  =  zeros(0);
                        end
                        
                        %Left side
                        if  tot_wdth < frm_tnrsct_arc_length_rghtlft
                            intrsct_strct.lft_wstbnd =  nodeId;
                            intrsct_strct.lft_estbnd =  nodeId + 1;
                            
                            fromEdges = [fromEdges intrsct_strct.lft_wstbnd ];
                            toEdges = [toEdges intrsct_strct.BotLeft ];
                            weights = [weights    enter_dist];
                            
                            fromEdges = [fromEdges intrsct_strct.TopLeft ];
                            toEdges = [toEdges intrsct_strct.lft_estbnd ];
                            weights = [weights    enter_dist];
                            
                            
                            [lx,ly] = roi{intrsct_strct.TopLeft}.get_rl_rgn().get_eq_x_max();
                            
                            roi{intrsct_strct.lft_wstbnd} = ...
                                obj.get_rd_rgn(f_rl,lx,ly, ...
                                enter_dist, -width, Rgn_of_Intrst_Enum.At, ...
                                Rd_Dir.Wst, tol);
                            
                            roi{intrsct_strct.lft_estbnd} = ...
                                obj.get_rd_rgn(f_rl, lx,ly, ...
                                enter_dist, width, Rgn_of_Intrst_Enum.Exiting, ...
                                Rd_Dir.Est, tol);
                            
                            
                            %[tlwx,tlwy] = roi{intrsct_strct.lft_wstbnd}.get_region_midpoint();
                            %[tlex,tley] = roi{intrsct_strct.lft_estbnd}.get_region_midpoint();
                            nodeId = nodeId +2;
                            
                        else
                            intrsct_strct.lft_wstbnd =  zeros(0);
                            intrsct_strct.lft_estbnd  =  zeros(0);
                        end
                        
                        %Top Side
                        if  tot_wdth < frm_tnrsct_arc_length_tpdwn
                            intrsct_strct.tp_nrthbnd =  nodeId;
                            intrsct_strct.tp_sthbnd  =  nodeId + 1;
                            
                            fromEdges = [fromEdges intrsct_strct.tp_sthbnd ];
                            toEdges = [toEdges intrsct_strct.TopLeft ];
                            weights = [weights    enter_dist];
                            
                            fromEdges = [fromEdges intrsct_strct.TopRight ];
                            toEdges = [toEdges intrsct_strct.tp_nrthbnd ];
                            weights = [weights    enter_dist];
                            
                            
                            [tx,ty] = roi{intrsct_strct.TopRight}.get_td_rgn().get_eq_y_max();
                            
                            roi{intrsct_strct.tp_nrthbnd} = ...
                                obj.get_rd_rgn(f_td, tx, ty, ...
                                enter_dist, width, Rgn_of_Intrst_Enum.Exiting, ...
                                Rd_Dir.Nrth, tol);
                            
                            roi{intrsct_strct.tp_sthbnd} = ...
                                obj.get_rd_rgn(f_td, tx, ty, ...
                                enter_dist, -width, Rgn_of_Intrst_Enum.At, ...
                                Rd_Dir.Sth, tol);
                            
                            %[tlnx,tlny] = roi{intrsct_strct.tp_nrthbnd}.get_region_midpoint();
                            %[trsx,trsy] = roi{intrsct_strct.tp_sthbnd}.get_region_midpoint();
                            nodeId = nodeId +2;
                            
                        else
                            intrsct_strct.tp_nrthbnd =  zeros(0);
                            intrsct_strct.tp_sthbnd  =  zeros(0);
                        end
                        
                        %Bottom Side
                        if  tot_wdth < to_intrsct_arc_length_tpdwn
                            intrsct_strct.bot_nrthbnd =  nodeId;
                            intrsct_strct.bot_sthbnd  =  nodeId + 1;
                            
                            fromEdges = [fromEdges intrsct_strct.bot_nrthbnd ];
                            toEdges = [toEdges intrsct_strct.BotRight];
                            weights = [weights enter_dist];
                            
                            fromEdges = [fromEdges intrsct_strct.BotLeft ];
                            toEdges = [toEdges intrsct_strct.bot_sthbnd ];
                            weights = [weights    enter_dist];
                            
                            [bx,by] = roi{intrsct_strct.BotRight}.get_td_rgn().get_eq_y_min();
                            roi{intrsct_strct.bot_nrthbnd} = ...
                                obj.get_rd_rgn(f_td, bx, by, ...
                                -enter_dist, width, Rgn_of_Intrst_Enum.At, ...
                                Rd_Dir.Nrth, tol);
                            
                            roi{intrsct_strct.bot_sthbnd} = ...
                                obj.get_rd_rgn(f_td, bx, by, ...
                                -enter_dist, -width, Rgn_of_Intrst_Enum.Exiting, ...
                                Rd_Dir.Sth,  tol);
                            
                            %[blnx, blny] = roi{intrsct_strct.bot_nrthbnd}.get_region_midpoint();
                            %[brsx, brsy] = roi{intrsct_strct.bot_sthbnd}.get_region_midpoint();
                            
                            nodeId = nodeId +2;
                            
                        else
                            intrsct_strct.bot_nrthbnd =  zeros(0);
                            intrsct_strct.bot_sthbnd  =  zeros(0);
                        end
                        if ~isKey(intrsct_mp, key)
                            intrsct_mp(key) = intrsct_strct;
                        else
                            intrsct_mp(key) = [intrsct_mp(key) intrsct_strct];
                        end
                    end
                end
            end
            
            for i = 1:1:numOfRoads
                eq = line_eqs{i};
                spln_sct = 1;
                intrsct_idx = intrsct_idx_mp(i);
                intrsct_rd_mtrx = intrsct_pos_mp(i);
                
                %For each of the intersections on the road, determine the
                %slope of the intersection and determine if it is the
                %right/left or the bottom/top
                prev_intrsct = zeros(0);
                prev_pos = [-1, -1];
                num_of_intrsct= intrsct_idx.getSize();
                first_pos = [roads(i).Centers(1,2) roads(i).Centers(1,1)];
                last_pos = [roads(i).Centers(end,2) roads(i).Centers(end,1)];
                if num_of_intrsct(1)== 0
                    continue
                end
                
                for j=1:1:num_of_intrsct(1)
                    
                    pos = intrsct_idx.getMatrixPos(j,1);
                    if i < pos
                        key  = [int2str(i) ',' int2str(pos)];
                    else
                        key  = [int2str(pos) ',' int2str(i)];
                    end
                    intrsct_strct = intrsct_mp(key);
                    if 1 < length(intrsct_strct)
                        num = intrsct_idx.which_occrncs(j, 1, pos);
                        intrsct_strct = intrsct_strct(num);
                    end
                    intrsct_x = intrsct_rd_mtrx.getMatrixPos(j, 1);
                    intrsct_y = intrsct_rd_mtrx.getMatrixPos(j, 2);
                    %intrsct_pos = [intrsct_x , intrsct_y];
                    spln_sct = 1;
                    f_i = line_eqs{i};
                    f_j = line_eqs{pos};
                    dirctn = determine_direction(f_i, f_j,intrsct_x, intrsct_y,tol);
                    
                    if dirctn == Intrsctn_Dir.EstWst
                        rght_lft =i;
                        tp_dwn = pos;
                        f_rl = line_eqs{i};
                        f_td = line_eqs{pos};
                    elseif dirctn == Intrsctn_Dir.NrthSth
                        rght_lft =pos;
                        tp_dwn = i;
                        f_rl = line_eqs{pos};
                        f_td = line_eqs{i};
                    end
                    
                    if isempty(prev_intrsct)
                        %First intersection of the road
                        
                        if  dirctn == Intrsctn_Dir.EstWst %Right
                            to_intrsct_arc_length_rghtlft = arc_length_of_spline_section(line_eqs{rght_lft}, ...
                                roads(rght_lft).Centers(1,2),  roads(rght_lft).Centers(1,1), intrsct_x, intrsct_y, tol);
                            
                            if tot_wdth < to_intrsct_arc_length_rghtlft
                                [rx,ry] = roi{intrsct_strct.rght_wstbnd}.get_eq_x_min();
                                indx = [intrsct_strct.rght_wstbnd intrsct_strct.rght_estbnd];
                            elseif width < to_intrsct_arc_length_rghtlft
                                [rx,ry] = roi{intrsct_strct.TopRight}.get_rl_rgn().get_eq_x_min();
                                indx = [intrsct_strct.BotRight intrsct_strct.TopRight];
                            else
                                [nxt_prev_x,nxt_prev_y]  = roi{intrsct_strct.lft_wstbnd}.get_eq_x_max();
                                prev_pos = [nxt_prev_x,nxt_prev_y];
                                prev_intrsct = intrsct_strct;
                                continue
                            end
                            
                            [spln_sct, nodeId, fromEdges, toEdges, weights, roi] = obj.update_graph_to_next_point( ...
                                1, line_eqs, i, spln_sct, first_pos, [rx,ry], ...
                                [-1 -1], indx, ...
                                [Rd_Dir.Est Rd_Dir.Wst], width, enter_dist, ...
                                nodeId, fromEdges, toEdges, weights, roi, tol);
                            
                            prev_intrsct = intrsct_strct;
                            [nxt_prev_x,nxt_prev_y]  = roi{intrsct_strct.lft_wstbnd}.get_eq_x_max();
                            prev_pos = [nxt_prev_x,nxt_prev_y];
                        elseif dirctn == Intrsctn_Dir.NrthSth %Bottom
                            to_intrsct_arc_length_tpdwn = arc_length_of_spline_section(line_eqs{tp_dwn}, ...
                                roads(tp_dwn).Centers(1,2),  roads(tp_dwn).Centers(1,1), intrsct_x, intrsct_y, tol);
                            
                            if tot_wdth < to_intrsct_arc_length_tpdwn
                                [bx,by] = roi{intrsct_strct.bot_nrthbnd}.get_eq_y_min();
                                indx = [intrsct_strct.bot_nrthbnd intrsct_strct.bot_sthbnd];
                            elseif width < to_intrsct_arc_length_tpdwn
                                [bx,by] = roi{intrsct_strct.BotRight}.get_td_rgn().get_eq_y_min();
                                indx = [intrsct_strct.BotRight intrsct_strct.BotLeft];
                            else 
                                [nxt_prev_x,nxt_prev_y]  = roi{intrsct_strct.tp_nrthbnd}.get_eq_y_max();
                                prev_pos = [nxt_prev_x,nxt_prev_y];
                                prev_intrsct = intrsct_strct;
                                continue
                            end
                            
                            [spln_sct, nodeId, fromEdges, toEdges, weights, roi] = obj.update_graph_to_next_point( ...
                                1, line_eqs, i, spln_sct, first_pos, [bx,by], ...
                                [-1 -1], indx, ...
                                [Rd_Dir.Nrth Rd_Dir.Sth], width, enter_dist, ...
                                nodeId, fromEdges, toEdges, weights, roi, tol);
                            
                            prev_intrsct = intrsct_strct;
                            [nxt_prev_x,nxt_prev_y]  =  roi{intrsct_strct.tp_nrthbnd}.get_eq_y_max();
                            prev_pos = [nxt_prev_x,nxt_prev_y];
                        end
                    else
                        
                        %Road segments connects two intersections
                        if dirctn == Intrsctn_Dir.EstWst %Right/Left
                            
                            [rx,ry] = roi{intrsct_strct.rght_wstbnd}.get_eq_x_min();
                            
                            [spln_sct, nodeId, fromEdges, toEdges, weights, roi] = obj.update_graph_to_next_point( ...
                                3, line_eqs, i, spln_sct, prev_pos, [rx,ry], ...
                                [prev_intrsct.lft_estbnd prev_intrsct.lft_wstbnd], ...
                                [intrsct_strct.rght_estbnd intrsct_strct.rght_wstbnd], ...
                                [Rd_Dir.Est Rd_Dir.Wst], width, enter_dist, ...
                                nodeId, fromEdges, toEdges, weights, roi, tol);
                            
                            prev_intrsct = intrsct_strct;
                            if ~isempty(intrsct_strct.lft_wstbnd)
                                [nxt_prev_x,nxt_prev_y]  =  roi{intrsct_strct.lft_wstbnd}.get_eq_x_max();
                                prev_pos = [nxt_prev_x,nxt_prev_y];
                            else
                                prev_pos = [];
                            end
                            
                        elseif dirctn == Intrsctn_Dir.NrthSth %Top/Bottom
                            [bx,by] = roi{intrsct_strct.bot_nrthbnd}.get_eq_y_min();
                            [spln_sct, nodeId, fromEdges, toEdges, weights, roi] = obj.update_graph_to_next_point( ...
                                3, line_eqs, i, spln_sct, prev_pos,  [bx,by], ...
                                [prev_intrsct.tp_nrthbnd prev_intrsct.tp_sthbnd], ...
                                [intrsct_strct.bot_nrthbnd intrsct_strct.bot_sthbnd], ...
                                [Rd_Dir.Nrth Rd_Dir.Sth], width, enter_dist, ...
                                nodeId, fromEdges, toEdges, weights, roi, tol);
                            
                            prev_intrsct = intrsct_strct;
                            if ~isempty(intrsct_strct.tp_nrthbnd)
                                [nxt_prev_x,nxt_prev_y]  =  roi{intrsct_strct.tp_nrthbnd}.get_eq_y_max();
                                prev_pos = [nxt_prev_x,nxt_prev_y];
                            else
                                prev_pos= [];
                            end
 
                        end        
                    end           
                end
                
                %Last intersection of the road
                if dirctn == Intrsctn_Dir.EstWst  %Left
                    
                    frm_tnrsct_arc_length_rghtlft =  arc_length_of_spline_section(line_eqs{rght_lft}, ...
                        intrsct_x, intrsct_y, roads(rght_lft).Centers(end,2), roads(rght_lft).Centers(end,1), tol);
                    
                    
                    if tot_wdth < frm_tnrsct_arc_length_rghtlft
                        [lx,ly] = roi{intrsct_strct.lft_wstbnd}.get_eq_x_max();
                        indx = [intrsct_strct.lft_wstbnd intrsct_strct.lft_estbnd];
                    elseif width < frm_tnrsct_arc_length_rghtlft
                        [lx,ly] = roi{intrsct_strct.TopLeft}.get_rl_rgn().get_eq_x_max();
                        indx = [intrsct_strct.BotLeft intrsct_strct.TopLeft];
                    else
                        continue
                    end
                    
                    [~, nodeId, fromEdges, toEdges, weights, roi] = obj.update_graph_to_next_point( ...
                        2, line_eqs, i, spln_sct, [lx,ly], last_pos, ...
                        indx, [-1 -1], ...
                        [Rd_Dir.Est Rd_Dir.Wst], width, enter_dist, ...
                        nodeId, fromEdges, toEdges, weights, roi, tol);
                    
                    
                elseif dirctn == Intrsctn_Dir.NrthSth %Top
                    
                    frm_tnrsct_arc_length_tpdwn =  arc_length_of_spline_section(line_eqs{tp_dwn}, ...
                        intrsct_x, intrsct_y, roads(tp_dwn).Centers(end,2), roads(tp_dwn).Centers(end,1), tol);
                    
                    if tot_wdth < frm_tnrsct_arc_length_tpdwn
                        [tx,ty] =  roi{intrsct_strct.tp_nrthbnd}.get_eq_y_max();
                        indx = [intrsct_strct.tp_nrthbnd intrsct_strct.tp_sthbnd];
                    elseif width < frm_tnrsct_arc_length_tpdwn
                         [tx,ty] = roi{intrsct_strct.TopRight}.get_td_rgn().get_eq_y_max();
                         indx = [intrsct_strct.TopRight intrsct_strct.TopLeft];
                    else
                        continue
                    end
                    
                    [~, nodeId, fromEdges, toEdges, weights, roi] = obj.update_graph_to_next_point( ...
                        2, line_eqs, i, spln_sct, [tx,ty], last_pos, ...
                        indx, [-1 -1], ...
                        [Rd_Dir.Nrth Rd_Dir.Sth], width, enter_dist, ...
                        nodeId, fromEdges, toEdges, weights, roi, tol);
                    
                end
            end
            
            graph = digraph(fromEdges,toEdges,weights);
        end
        
        %% Update the graph by finding the next interesting point
        %
        % [SPLN_SCT, NODEID, FROMEDGES, TOEDGES, WEIGHTS, ROI] =
        % UPDATE_GRAPH_TO_NEXT_POINT(OBJ, ROAD_ST, LINE_EQS, RD_INDX,
        % SPLN_SCT, FIRST_POS, LAST_POS,FIRST_INDX, LAST_INDX, RD_DIRS,
        % ROAD_WIDTH, ENTER_DIST,NODEID, FROMEDGES, TOEDGES, WEIGHTS, ROI,
        % TOL)  - Follows the roadway and creates the next region of
        % interest for any entry points, exit points and curve apex while 
        % creating any edge connections 
        %
        % INPUTS 
        %
        % ROAD_ST - State of the road: Starting of road, ending of road, other
        % LINE_EQS - the models of the roads
        % RD_INDX -  the index of the road
        % SPLN_SCT - the current section on the spline of the road
        % FIRST_POS - the starting position of the road
        % LAST_POS - the ending position of the road
        % FIRST_INDX - the starting index
        % LAST_INDX - the final index
        % RD_DIRS - The directions of the road
        % ROAD_WIDTH - The width of the road
        % ENTER_DIST - The enter distance from the intersection
        % NODEID - The current node ID
        % FROMEDGES - the set of "from" edges
        % TOEDGES - the set of "to" edges
        % WEIGHTS - the set of weights for the graph
        % ROI - the regions of interest for each node
        % TOL - the tolerance for the number of points used to model a spline
        %
        % OUTPUTS
        %
        % SPLN_SCT - The new section on the spline
        % NODEID - The new node ID
        % FROMEDGES - the set of "from" edges
        % TOEDGES - the set of "to" edges
        % WEIGHTS - the set of weights for the graph
        % ROI - the regions of interest for each node
        
        function [spln_sct, nodeId, fromEdges, toEdges, weights, roi] = update_graph_to_next_point( ...
                obj, road_st, line_eqs, rd_indx, spln_sct, first_pos, last_pos, ...
                first_indx, last_indx, rd_dirs, road_width, enter_dist, ...
                nodeId, fromEdges, toEdges, weights, roi, tol)
            
            if isempty(first_indx) || isempty(last_indx) %end of road check
                return
            end
            total_dist = road_width + enter_dist;
            f_rd = line_eqs{rd_indx};
            next_point = [first_pos(1) first_pos(2)];
            previous_point = [first_pos(1) first_pos(2)];
            prev_indx = [first_indx(1) first_indx(2)];
            total_arc_length = arc_length_of_spline_section(f_rd, ...
                first_pos(1), first_pos(2), last_pos(1), last_pos(2),tol);
            
            while ~(next_point(1) == last_pos(1) &&  next_point(2) == last_pos(2))
                [next_point,spln_sct] = get_next_important_point_on_spline(f_rd,spln_sct, next_point, last_pos, tol);
                
                arc_length = arc_length_of_spline_section(f_rd, ...
                    previous_point(1), previous_point(2), next_point(1), next_point(2),tol);
                
                % Beginning of the Road to Intersection
                if (road_st == 1) && previous_point(1) == first_pos(1) && previous_point(2) == first_pos(2)
                    %if total_arc_length <= total_dist
                    %    return
                    %else
                    
                    if (next_point(1) == last_pos(1) &&  next_point(2) == last_pos(2))
                        if rd_dirs(1) == Rd_Dir.Est || rd_dirs(1) == Rd_Dir.Wst
                            currFromEdge = [(nodeId) last_indx(1)];
                            currToEdge   = [last_indx(2) (nodeId+ 1)];
                        else
                            currFromEdge = [(nodeId) last_indx(2)];
                            currToEdge   = [last_indx(1) (nodeId+ 1)];
                        end
                        rois = [Rgn_of_Intrst_Enum.EndOfRoad Rgn_of_Intrst_Enum.EndOfRoad];
                        roi_points = first_pos;
                        rd_wdth_mod = [1 -1];
                        dir_mod = -1;
                    else
                        %for when we have a curve and the end of the road
                        %first, we add two points in the first pass
                        rd_wdth_mod = [1 -1];
                        dir_mod = -1;
                        rois = [Rgn_of_Intrst_Enum.EndOfRoad Rgn_of_Intrst_Enum.EndOfRoad];
                        roi_points = first_pos;
                        
                        
                        roi{nodeId} = ...
                            obj.get_rd_rgn(f_rd, roi_points(1), roi_points(2), ...
                            dir_mod*enter_dist, (rd_wdth_mod(1))*road_width, rois(1), ...
                            rd_dirs(1), tol);

                        roi{nodeId+1} = ...
                            obj.get_rd_rgn(f_rd, roi_points(1), roi_points(2), ...
                            dir_mod*enter_dist, (rd_wdth_mod(2))*road_width, rois(2), ...
                            rd_dirs(2),  tol);

                        
                        rois = [Rgn_of_Intrst_Enum.CurveInRoad Rgn_of_Intrst_Enum.CurveInRoad];
                        roi_points = next_point;
                        prev_indx = [(nodeId) (nodeId+1)];
                        nodeId = nodeId +2;
                        
                        if rd_dirs(1) == Rd_Dir.Est || rd_dirs(1) == Rd_Dir.Wst
                            currFromEdge = [prev_indx(1) (nodeId) ];
                            currToEdge   = [(nodeId+1) prev_indx(2) ];
                        else
                            currFromEdge = [prev_indx(1)  (nodeId+1) ];
                            currToEdge   = [(nodeId) prev_indx(2) ];
                        end                    
                        
                    end
                        
                       
                    %end
                    
                    % Intersection to End of the Road
                elseif (road_st == 2) && next_point(1) == last_pos(1) && next_point(2) == last_pos(2)
                    %if total_arc_length <= total_dist
                    %    return
                    %else
                    if rd_dirs(1) == Rd_Dir.Est || rd_dirs(1) == Rd_Dir.Wst
                        currFromEdge = [prev_indx(2) (nodeId)];
                        currToEdge   = [(nodeId + 1) prev_indx(1)];
                        rd_wdth_mod = [-1 1];
                    else
                        currFromEdge = [prev_indx(1) (nodeId)];
                        currToEdge   = [(nodeId + 1) prev_indx(2)];
                        rd_wdth_mod = [-1 1];
                    end
                    rois = [Rgn_of_Intrst_Enum.EndOfRoad Rgn_of_Intrst_Enum.EndOfRoad];
                    dir_mod = 1;
                    roi_points = next_point;
                    %end
                    % In between intersections
                else 
                    if next_point(1) == last_pos(1) && next_point(2) == last_pos(2)
                        fromEdges = [fromEdges prev_indx(1) ];
                        toEdges = [toEdges last_indx(1) ];
                        weights = [weights arc_length];
                        
                        fromEdges = [fromEdges last_indx(2) ];
                        toEdges = [toEdges prev_indx(2) ];
                        weights = [weights arc_length];
                        
                        return;
                        
                    else
                        currFromEdge = [prev_indx(1) (nodeId)];
                        currToEdge   = [(nodeId + 1) prev_indx(2)];
                        rois = [Rgn_of_Intrst_Enum.CurveInRoad Rgn_of_Intrst_Enum.CurveInRoad];
                        roi_points = next_point;
                        dir_mod = 1;
                    end
                    
                    if rd_dirs(1) == Rd_Dir.Est || rd_dirs(1) == Rd_Dir.Wst
                        rd_wdth_mod = [1 -1];
                    else
                        rd_wdth_mod = [-1 1];
                    end
                end
                fromEdges = [fromEdges currFromEdge(1) ];
                toEdges = [toEdges currToEdge(1) ];
                weights = [weights arc_length];
                
                fromEdges = [fromEdges currFromEdge(2)];
                toEdges = [toEdges currToEdge(2) ];
                weights = [weights arc_length];

                roi{nodeId} = ...
                    obj.get_rd_rgn(f_rd, roi_points(1), roi_points(2), ...
                    dir_mod*enter_dist, (rd_wdth_mod(1))*road_width, rois(1), ...
                    rd_dirs(1), tol);
                
                roi{nodeId+1} = ...
                    obj.get_rd_rgn(f_rd, roi_points(1), roi_points(2), ...
                    dir_mod*enter_dist, (rd_wdth_mod(2))*road_width, rois(2), ...
                    rd_dirs(2),  tol);
                
                prev_indx = [(nodeId) (nodeId + 1)];
                nodeId = nodeId +2;
                previous_point = roi_points;
            end
            
        end
        
        %% Create a region of interest for an intersection
        %
        % NODE = GET_INTRSCTN_RGN(OBJ, F_RL, F_TD, X_POS, 
        % Y_POS, RL_X_SGN, RL_Y_SGN, 
        % TD_X_SGN, TD_Y_SGN, LENGTH, 
        % ROAD_WIDTH, DIR_X, DIR_Y, QUAD, TOL) - create a region of interest based on F_RL
        % and F_TD using RL_X_SGN/TD_X_SGN, RL_Y_SGN/TD_Y_SGN, DIR_X/DIR_Y and ROAD_WIDTH/LENGTH to calculate
        % the bounding area of the ROI
        %
        % OUTPUTS
        %
        % NODE the region of interest object that contains the bounding area information
        %
        function node = get_intrsctn_rgn(obj, f_rl,f_td, x_pos, y_pos, rl_x_sgn, rl_y_sgn, td_x_sgn, td_y_sgn, length, road_width,dir_x, dir_y, quad, tol)
            rl_rgn = get_rd_rgn(obj, f_rl, x_pos, y_pos, rl_x_sgn*length, rl_y_sgn*road_width, Rgn_of_Intrst_Enum.Within, dir_x, tol);
            td_rgn = get_rd_rgn(obj, f_td, x_pos, y_pos, td_y_sgn*length, td_x_sgn*road_width, Rgn_of_Intrst_Enum.Within, dir_y, tol);
            node = Intrsctn_Rgn(Rgn_of_Intrst_Enum.Within, rl_rgn,td_rgn, quad);
        end
        

        %% Create a region of interest for a road
        %
        % NODE = GET_RD_RGN(OBJ, F, X_POS, 
        % Y_POS, LENGTH, ROAD_WIDTH, DIR, TOL) - create a region of interest based on F
        % using DIR ROAD_WIDTH/LENGTH to calculate
        % the bounding area of the ROI
        %
        % OUTPUTS
        %
        % NODE the region of interest object that contains the bounding area information
        %
        function node = get_rd_rgn(obj, f, x_pos, y_pos, length, road_width, intsct_type, dir, tol)
            [~,x2,y2] = estimate_xy_after_length(f, x_pos,y_pos,length, tol);
            
            if intsct_type  == Rgn_of_Intrst_Enum.EndOfRoad
                node = Rd_Rgn(intsct_type, f, x_pos,y_pos, ...
                    x_pos, y_pos, road_width, dir,  tol);
            elseif intsct_type  == Rgn_of_Intrst_Enum.CurveInRoad
                node = Rd_Rgn(intsct_type, f, x_pos,y_pos, ...
                    x_pos, y_pos, road_width, dir,  tol);
            else
                node = Rd_Rgn(intsct_type, f, x_pos,y_pos, ...
                    x2, y2, road_width, dir,  tol);
            end
        end
        
        
        
        %% Get the path for a turn
        %
        % [X, Y] = GET_NEXT_PATH_FOR_TURN(OBJ, ROI, DIR) Get the ending point for
        % a specific region of interest (ROI) and a specific direction of travel (DIR)
        %
        % OUTPUTS
        %
        % X,Y - the x, y point
        %
        function [x,y] = get_next_path_for_turn(obj, roi, dir)
            if (dir == Rd_Dir.Est)
                [x,y] = roi.get_furthest_eastern_point();
            elseif (dir == Rd_Dir.Sth)
                [x,y] = roi.get_furthest_southern_point();
            elseif (dir == Rd_Dir.Wst)
                [x,y] = roi.get_furthest_western_point();
            elseif (dir == Rd_Dir.Nrth)
                [x,y] = roi.get_furthest_northern_point();
            end
        end
        

        %% Get the path for a turn
        %
        % TRN_TYPE = GET_TURN_TYPE(OBJ, FROM_DIR, TO_DIR) get the type of turn based on
        % The direction of travel orginally (FROM_DIR) and after the turn (TO_DIR)
        %
        % OUTPUTS
        %
        % TRN_TYPE - the type of turn, either right turn, left turn or moving straight
        %

        function trn_type = get_turn_type(obj, from_dir, to_dir)
            if  (from_dir == Rd_Dir.Est && to_dir == Rd_Dir.Nrth) || ...
                    (from_dir == Rd_Dir.Sth && to_dir == Rd_Dir.Est)  ||...
                    (from_dir == Rd_Dir.Wst && to_dir == Rd_Dir.Sth)  || ...
                    (from_dir == Rd_Dir.Nrth && to_dir ==Rd_Dir.Wst)%RIGHT TURN
                trn_type= Vehicle_Movement.Turning_Right;
            elseif (from_dir == to_dir) %STRAIGHT
                trn_type = Vehicle_Movement.Driving_Straight;
            elseif (from_dir == Rd_Dir.Nrth && to_dir == Rd_Dir.Est) || ...
                    (from_dir == Rd_Dir.Est && to_dir ==Rd_Dir.Sth)  ||...
                    (from_dir == Rd_Dir.Sth && to_dir == Rd_Dir.Wst)  || ...
                    (from_dir == Rd_Dir.Wst && to_dir == Rd_Dir.Nrth)%LEFT TURN
                trn_type= Vehicle_Movement.Turning_Left;
            end
            
        end
        
        
    end
end
