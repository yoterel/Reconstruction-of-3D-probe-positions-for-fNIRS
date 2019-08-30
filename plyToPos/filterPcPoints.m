function filteredPc = filterPcPoints(pc, mask)
%FILTERPCPOINTS Returns a pointCloud object resulting from applying the
%given mask on the given pointCloud's vertices, while preserving the other
%properties in the pointCloud (such as normals, colors etc')
n = size(pc.Location, 1);
vertices = returnFilteredArr(pc.Location, mask, n);
colors = returnFilteredArr(pc.Color, mask, n);
normals = returnFilteredArr(pc.Normal, mask, n);
intensities = returnFilteredArr(pc.Intensity, mask, n);
filteredPc = pointCloud(vertices, 'Color', colors, 'Normal', normals, 'Intensity', intensities);
end

function [filtered] = returnFilteredArr(arr, mask, n)
if (size(arr, 1) == n)
    filtered = arr(mask, :);
else
    filtered = [];
end
end
