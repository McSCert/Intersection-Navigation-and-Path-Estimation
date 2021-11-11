%% Convert a spline into a symbolic equation
% F_SET = SPLINE2SYM(SPLINE)
% Converts a spline (SPLINE) into a set of 
% functions (F_SET)

function f_set = spline2sym(spline)
    syms x
    f_set = {};
    
    
    for i =1:1:(spline.p.pieces)
        x1  = spline.p.breaks(i);
        order = spline.p.order;
        eq  = [];
        for j = 1:1:order-1
            if isempty(eq)
               eq  = spline.p.coefs(i,j)*(x - x1)^(order-j); 
            else
               eq  = eq + spline.p.coefs(i,j)*(x - x1)^(order-j); 
            end
        end
        eq = eq + spline.p.coefs(i,order);
        f_set{i} = eq;
    end
end

