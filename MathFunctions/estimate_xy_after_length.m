%% Estimate the point after a given arclength
%  [PIECE,X1,Y1] = ESTIMATE_XY_AFTER_LENGTH(SPLINE, X0,Y0,L, TOL)
% Estimate the point after a given arclength (L) for a given spline 
% (SPLINE), starting at position (X0,Y0)
%
% OUTPUT
%
% PIECE - the number that corresponds to a piece of the spline
% X1,Y1 - the estimated position
% 
function [piece,x1,y1] = estimate_xy_after_length(spline, x0,y0,L, tol)
 [piece, pos, x_rng, y_rng] = find_position_in_spline(spline, x0, y0, tol);
 dir = sign(L);
 L = abs(L);
 length_not_met = true;
 prev_arc_length = 0;
 arc_length = 0;
 while length_not_met
    if dir ==1
        for i = (pos+1):dir:length(x_rng)
            arc_length = arclength(x_rng(pos:i),y_rng(pos:i));
            if L < abs(prev_arc_length + dir*arc_length) 
                x1= x_rng(i);
                y1= y_rng(i);
                return
            end
        end
    else
        for i = (pos-1):dir:1
            arc_length = arclength(x_rng(i:pos),y_rng(i:pos));
            if L < abs(prev_arc_length + dir*arc_length) 
                x1= x_rng(i);
                y1= y_rng(i);
                return
            end
        end
    end
    
    
    prev_arc_length = prev_arc_length + dir*arc_length;
    piece = piece +1;
    
    if (spline.points - 1)  < piece
        %Past the last point of the spline, using first or last depending
        %on direction
        if dir ==1
           x1= x_rng(end);
           y1= y_rng(end);
           return
        else
           x1= x_rng(1);
           y1= y_rng(1);
           return
        end
    else
        [x_rng, y_rng] = pointsOnClothoid(spline.x(piece), spline.y(piece), ...
            spline.theta(piece), spline.k(piece), spline.dk(piece), spline.L(piece), 1/tol);
    end
    
 end
end

