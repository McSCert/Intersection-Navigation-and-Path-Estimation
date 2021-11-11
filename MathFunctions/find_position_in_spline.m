%% Find the point in a spline
% [PIECE, POS, X_RNG, Y_RNG] = FIND_POSITION_IN_SPLINE(SPLINE, X, Y, TOL)
% Estimates the position in a spline (SPLINE)
% at a point of (X, Y) with a tolerance of TOL 
%
% OUTPUT
%
% X_RNG, Y_RNG - the x, y points along the clothoid
% POS - the position in the clothoid
% PIECE - the number that coresponds to what piece within the spline
function [piece, pos, x_rng, y_rng] = find_position_in_spline(spline, x, y, tol)
for i = 1:1:(spline.points-1)
    [pos,x_rng, y_rng] = find_position_in_clothoid(spline,i, x, y, tol);
    if pos ~= -Inf
        piece = i;
        return;
    end
    
end
error('The point was not found in the spline');
pos = -inf;
piece = -inf;
x_rng = [];
y_rng = [];
end