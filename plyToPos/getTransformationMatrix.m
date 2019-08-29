function [mat] = getTransformationMatrix(scale, translate, rotate)
%GETTRANSFORMATIONMATRIX Gets a 4 X 4 transformation matrix for the given
%scaling, translation and (optionally) rotation (default is no rotation).
%   INPUT:
%           scale: a scalar
%       translate: a 1 X 3 vector
%          rotate: a 3 X 3 matrix
if nargin < 3
    rotate = eye(3);
end
mat = [scale * rotate;translate];
finalCol = [zeros(3, 1);1];
mat = [mat, finalCol];
end

