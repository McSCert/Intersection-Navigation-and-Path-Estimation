%Determines if (x1,y1) < (x2, y2)
% FUNCTION RESULT = COOR_LESSTHEN(X1,Y1, X2, Y2)
% checks to see if (X1,Y1) < (X2, Y2) first based on
% X1 < X2 then based on Y1 < Y2
%
% OUTPUT
%
% RESULT -  boolean value for if (x1,y1) < (x2, y2) 
function result = coor_lessthen(x1,y1, x2, y2)
if (abs(x1-x2)< 10e-10)
    if y1 < y2
        result = true;
    elseif y2 <= y1
        result = false;
    end
elseif x1 < x2 
    result = true;
elseif x2 < x1 
    result = false;
else
    error('This is not possible')
end
end