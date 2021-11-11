%% Determine a direction for a road based on two road models
%  DIRCTN = DETERMINE_DIRECTION(F_MAIN, F_COMP, X, Y, TOL)
% Determine a direction for a road based on two road models
% (F_MAIN, F_COMP), at an intersection point of (X, Y) and
% a tolerance of TOL
%
% OUTPUT
%
% DIRCTN - the direction of the road that is modeled by F_MAIN
%
function dirctn = determine_direction(f_main, f_comp, x, y, tol)
%find the slopes of the two intersecting lines,
%determine if it is the left/right entrance or
%top/bottom
road_1_slope =  estimate_derivative_clothoid(f_main, x,y, tol);
road_2_slope =  estimate_derivative_clothoid(f_comp, x,y, tol);

if road_1_slope < road_2_slope
    dirctn = Intrsctn_Dir.EstWst; %WEST-BOUND/EAST-BOUND
elseif road_2_slope < road_1_slope
    dirctn = Intrsctn_Dir.NrthSth; % NORTH-BOUND/SOUTH-BOUND
else
    error('Same slope for given lines')
end
end
