function [cleanedPc] = pcRemoveOutliers(pc, stdev)
%PCREMOVEOUTLIERS Cleans points that are too far from the center of mass in
% the given point cloud (optional argument: the maximal standard deviation
% of the distances of the points from the center of mass, default is 2).
if (nargin < 2)
    stdev = 2;
end
n = size(pc, 1);
centerOfMass = sum(pc) / n;
distances = sqrt(sum((pc-centerOfMass).^2, 2));
normalized = normalize(distances);
cleanedPc = pc(abs(normalized) < stdev, :);
end