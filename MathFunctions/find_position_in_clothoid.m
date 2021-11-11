%% Find the point in a clothoid
%  [POS, X_RNG, Y_RNG] = FIND_POSITION_IN_CLOTHOID(SPLINE,I, X, Y, TOL)
% Estimates the position in a clothoid (spline (SPLINE) at section (I))
% at a point of (X, Y) with a tolerance of TOL 
%
% OUTPUT
%
% X_RNG, Y_RNG - the x, y points along the clothoid
% POS - the position in the clothoid
%
function [pos, x_rng, y_rng] = find_position_in_clothoid(spline,i, x, y, tol)
npnt_a = ceil(1/tol);
[x_rng, y_rng] = pointsOnClothoid(spline.x(i), spline.y(i), ...
    spline.theta(i), spline.k(i), spline.dk(i), spline.L(i), npnt_a);
pos = -inf;
for j = 1:1:(npnt_a-1)
    x_in_rng = (x_rng(j) <= x && x <= x_rng(j+1)) ||...
        (x_rng(j+1) <= x && x <= x_rng(j)) || ...
        (abs(x_rng(j) - x)  < 10e-6);
    y_in_rng = (y_rng(j) <= y && y <= y_rng(j+1)) ||...
        (y_rng(j+1) <= y && y <= y_rng(j)) || ...
        (abs(y_rng(j) - y)  < 10e-6);
    if x_in_rng && y_in_rng
        pos = j;
        return
    end
end

%try
if (abs(x_rng(npnt_a) - x)  < 10e-6) && (abs(y_rng(npnt_a) - y)  < 10e-6)
    pos = npnt_a;
    return;
end
%catch e
%error('not found');
%end
end

