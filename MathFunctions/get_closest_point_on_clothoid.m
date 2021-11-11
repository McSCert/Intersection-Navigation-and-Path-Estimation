%% Find the closest point on a clothoid
% [X_MIN,Y_MIN]= GET_CLOSEST_POINT_ON_CLOTHOID(POINTS, X,Y)
% Find the closest point to an X, Y point for a given set of
% points along the clothoid (POINTS)
%
% OUTPUT
%
% X_MIN, Y_MIN - the closest x, y point along the clothoid
%
function [x_min,y_min]= get_closest_point_on_clothoid(points, x,y)
distances = sqrt(sum(bsxfun(@minus, points, [x y]).^2,2));
closest = points(find(distances==min(distances)),:);
x_min = closest(1, 1);
y_min = closest(1, 2);
end

