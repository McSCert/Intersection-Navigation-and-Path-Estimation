%% Get the arc length for a section of the spline
% ARC_LENGTH = ARC_LENGTH_OF_SPLINE_SECTION(SPLINE, X_A, Y_A, X_B, Y_B, TOL)
% Find the arclength of a given spline (SPLINE), based on two points 
% (X_A, Y_A), (X_B, Y_B) and a given tolerance (TOL)
% 
%
% OUTPUT
%
% ARC_LENGTH - the arc length along the spline
%
function arc_length = arc_length_of_spline_section(spline, x_a, y_a, x_b, y_b, tol)
    [piece_a, pos_a] = find_position_in_spline(spline, x_a, y_a, tol);
    [piece_b, pos_b] = find_position_in_spline(spline, x_b, y_b, tol);
    if piece_b < piece_a
        tmp_pc = piece_a;
        tmp_pos = pos_a;
        piece_a  = piece_b;
        pos_a  = pos_b;
        piece_b = tmp_pc;
        pos_b = tmp_pos;
    end
    if piece_a == piece_b 
        if pos_b < pos_a
            tmp = pos_a;
            pos_a = pos_b;
            pos_b = tmp;
        end
        arc_length= get_arc_length_of_clothoid_section(spline,...
            piece_a,pos_a, pos_b ,tol);
    else
       

        %Starting section of the clothoid
        arc_length= get_arc_length_of_clothoid_section(spline,...
            piece_a,pos_a, inf ,tol);

        %middle sections, all of the middle sections will be the full length of
        %the clothoid
        for i = (piece_a+1):1:(piece_b-1)
          arc_length = arc_length + ... 
              get_arc_length_of_clothoid_section(spline, piece, inf,-1, tol);
        end
        %Ending section of the clothoid
        arc_length = arc_length + ...
            get_arc_length_of_clothoid_section(spline, piece_b, -1,pos_b, tol);
    end
end


%% Get the arc length for a section of the clothoid
% ARC_LENGTH= GET_ARC_LENGTH_OF_CLOTHOID_SECTION(SPLINE, PIECE, POS_START,POS_END, TOL)
% Find the arclength of a given clothoid (for piece (PIECE) for the spline (SPLINE), 
% based on a starting and ending positions (POS_START,POS_END)  
% and a given tolerance (TOL)
%
% OUTPUT
%
% ARC_LENGTH - the arc length along the piece of the clothoid
%
function arc_length= get_arc_length_of_clothoid_section(spline, piece, pos_start,pos_end, tol)
    npnt_a = 1/tol;
    [px, py] = pointsOnClothoid(spline.x(piece), spline.y(piece), ...
    spline.theta(piece), spline.k(piece), spline.dk(piece), spline.L(piece), npnt_a);

    pos_start = max(1,pos_start);
    pos_end = min(length(px),pos_end);
    if pos_end - pos_start < 1
        arc_length = 0;
    else
        [arc_length,~] = arclength(px(pos_start:pos_end),py(pos_start:pos_end));
    end
end