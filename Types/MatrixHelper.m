classdef MatrixHelper
    %SHAPE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access=private)
        mtrx        
    end
    
    methods
        function obj = MatrixHelper()
            %MapHelper Construct an instance of this class
            %   Detailed explanation goes here
            obj.mtrx = [];
        end
        
        function obj = setMatrix(obj,mtrx)
            %Matrix Set the matrix of the object
            obj.mtrx =  mtrx;
        end
        
        function obj = addRow(obj, row, pos)
            size = obj.getSize();
            if 1 < size(2)
                obj.mtrx = [obj.mtrx(1:pos-1, :); row; obj.mtrx(pos:end, :)];
            else
                obj.mtrx = [obj.mtrx(1:pos-1); row; obj.mtrx(pos:end)];
            end
        end
        
        function value = getMatrixPos(obj, x, y)
            value = obj.mtrx(x,y);
        end
        
        function sizeOfMatrix = getSize(obj)
            sizeOfMatrix = size(obj.mtrx);
        end
        function pos = findPos(obj, value)
            pos = find(obj.mtrx, value);
        end
        
        function out = isEmpty(obj)
            out = isempty(obj.mtrx);
        end
        
        function out = num_of_occrncs(obj, col, item)
            column= obj.mtrx(:,col);
            out = length(column(column==item));
        end
        
        function out = which_occrncs(obj, row, col, item)
            out =1;
            for i=1:1:row-1
                if obj.mtrx(i,col) ==item
                    out = out + 1;
                end
            end
        end
        function dspMatrix(obj)
           obj.mtrx 
        end
    end
end

