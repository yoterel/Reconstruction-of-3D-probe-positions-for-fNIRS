function [transformed] = applyRegParams(points, regParams, invert)
%APPLYREGPARAMS transforms the given set of points using the given
% regParams struct (usually returned from absor)
transformationMatrix = regParams.M';
if (nargin > 2 && invert)
    transformationMatrix = transformationMatrix^-1;
end
transformed = [points, ones(size(points, 1), 1)] * transformationMatrix;
transformed = transformed(:, 1:3);
end