%% Get the next important point along a spline
% [NEXT_PT,PT] = GET_NEXT_IMPORTANT_POINT_ON_SPLINE(F, PT,SRT_PT, END_PT, TOL)
% Find the next curving or ending point along a spline (F) starting at 
% an x, y point (SRT_PT) and an ending x, y point (END_PT) 
% for a tolerance of TOL
%
% OUTPUT
%
% NEXT_PT - the next curving point or ending point in a clothoid
% PT - the number that coresponds to what piece within the spline

function [next_pt,pt] = get_next_important_point_on_spline(f, pt,srt_pt, end_pt, tol)
    
    try
        [pos, x_rng, y_rng] = find_position_in_clothoid(f, pt,srt_pt(1), srt_pt(2), tol);
    catch 
        next_pt = end_pt;
        return
    end
    
    if pos == -Inf
        pt = pt+1;
        next_pt(1) = x_rng(end);
        next_pt(2) = y_rng(end);
        return
    end
    
    for i = pos:1:length(x_rng)
        if abs(x_rng(i) - end_pt(1)) < 10e-10 && abs(y_rng(i) - end_pt(2))< 10e-10
            next_pt = end_pt;
            return
        end
    end
    
    pt = pt+1;
    next_pt(1) = x_rng(end);
    next_pt(2) = y_rng(end);
end

