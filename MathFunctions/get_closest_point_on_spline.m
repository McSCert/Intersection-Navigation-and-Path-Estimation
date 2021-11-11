%% Find the closest point on a spline
% [X_MIN, Y_MIN, PIECE] = GET_CLOSEST_POINT_ON_SPLINE(SPLINE, X, Y, TOL)
% Find the closest point to an X, Y point for a given set of
% points along a spline (SPLINE) with a tolerance of TOL
%
% OUTPUT
%
% X_MIN, Y_MIN - the closest x, y point along the spline
% PIECE - the number that coresponds to what piece within the spline
function [x_min, y_min, piece] = get_closest_point_on_spline(spline, x, y, tol)
min_dist = Inf;
for i = 1:1:(spline.points - 1)
    
    npnt_a = 1/tol;
    [x_rng, y_rng] = pointsOnClothoid(spline.x(i), spline.y(i), ...
        spline.theta(i), spline.k(i), spline.dk(i), spline.L(i), npnt_a);
    
    [x_min_sect, y_min_sect] = get_closest_point_on_clothoid([x_rng; y_rng]', x, y);
    distances = sqrt(sum(bsxfun(@minus, [x_min_sect, y_min_sect], [x y]).^2,2));
    if distances < min_dist
        piece = i;
        x_min = x_min_sect;
        y_min = y_min_sect;
        min_dist = distances;
    end

end
end

