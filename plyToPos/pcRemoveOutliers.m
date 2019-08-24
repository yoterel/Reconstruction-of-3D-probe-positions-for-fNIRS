function [cleanedPc] = pcRemoveOutliers(pc, stdev)
%PCREMOVEOUTLIERS Cleans points that are too far from the center of mass in
% the given point cloud (optional argument: the maximal standard deviation
% of the distances of the points from the center of mass, default is 2).
if (nargin < 2)
    stdev = 2;
end
vertices = pc;
isPointCloudObj = strcmp(class(pc), 'pointCloud');
if (isPointCloudObj)
    vertices = pc.Location;
end
n = size(vertices, 1);
centerOfMass = sum(vertices) / n;
distances = sqrt(sum((vertices - centerOfMass).^2, 2));
normalized = normalize(distances);
mask = abs(normalized) < stdev;
if (isPointCloudObj)
    vertices = returnFilteredArr(vertices, mask, n);
    colors = returnFilteredArr(pc.Color, mask, n);
    normals = returnFilteredArr(pc.Normal, mask, n);
    intensities = returnFilteredArr(pc.Intensity, mask, n);
    cleanedPc = pointCloud(vertices, 'Color', colors, 'Normal', normals, 'Intensity', intensities);
else
    cleanedPc = pc(mask, :);
end
end

function [filtered] = returnFilteredArr(arr, mask, n)
if (size(arr, 1) == n)
    filtered = arr(mask, :);
else
    filtered = [];
end
end