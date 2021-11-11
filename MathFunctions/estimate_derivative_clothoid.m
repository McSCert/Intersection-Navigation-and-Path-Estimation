%% Estimate the derivative of the clothoid 
%  DIRV = ESTIMATE_DERIVATIVE_CLOTHOID(F, X, Y, TOL)
% Estimate the derivative of the clothoid (F) at a points
% (X, Y) for a tolernace of (TOL)
%
% OUTPUT
%
% DIRV - the slope of the clothoid
% 
function dirv = estimate_derivative_clothoid(f, x, y, tol)
for i=1:1:(f.points-1)
        [pos, x_rng, y_rng] = find_position_in_clothoid(f,i, x, y, tol);
        if pos == -Inf
            continue;
        elseif pos ~= length(y_rng)
            dirv = (y_rng(pos+1)- y_rng(pos))/(x_rng(pos+1)- x_rng(pos));
            return
        elseif pos ~=0
            dirv = (y_rng(pos)- y_rng(pos-1))/(x_rng(pos)- x_rng(pos-1));
            return
        end
end
error('Position not found');
end
