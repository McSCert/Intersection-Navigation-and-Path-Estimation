%% Find the intersection points between two splines
%  [X_COMMON, Y_COMMON] = DTRMN_INTRSCTS_BTWN_SPLINES(F_A, F_B ,TOL)
% Find the intersection points between two splines (F_A, F_B) with a
% search tolerance of TOL
%
% OUTPUT
%
% [X_COMMON, Y_COMMON] - the x and y points that coorespond to intersection
% points
function [x_common, y_common] = dtrmn_intrscts_btwn_splines(f_a, f_b ,tol)
x_common = [];
y_common = [];
for i=1:1:(f_a.points-1)
    for j = 1:1:(f_b.points-1)
        npnt_a = 1/tol;
        npnt_b = 1/tol;
        [x_a, y_a] = pointsOnClothoid(f_a.x(i), f_a.y(i), ...
            f_a.theta(i), f_a.k(i), f_a.dk(i), f_a.L(i), npnt_a);
        
        [x_b, y_b] = pointsOnClothoid(f_b.x(j), f_b.y(j), ...
            f_b.theta(j), f_b.k(j), f_b.dk(j), f_b.L(j), npnt_b);
        
        P = InterX([x_a;y_a],[x_b;y_b]);
        
        [n,m] = size(P);
        for k  = 1:1:m
            unique = true;
            for z = 1:1:length(x_common)
                if abs(x_common(z) -  P(1,k)) < 10e-10 && ...
                        abs(y_common(z) -  P(2,k)) < 10e-10
                    unique = false;
                end
            end
            if unique
                x_common =[x_common P(1,k)];
                y_common =[y_common P(2,k)];
            end
        end
    end
end
end
