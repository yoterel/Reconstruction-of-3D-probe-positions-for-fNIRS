function [pc] = structToPointCloud(pcStruct, convertColors)
%STRUCTTOPOINTCLOUD Converts a struct returned from plyread into a
%pointCloud object
v = pcStruct.vertex;
normals = [v.nx, v.ny, v.nz];
pc = pointCloud([v.x, v.y, v.z], 'Normal', normals);
if nargin < 2 || convertColors
    pc.Color = uint8([v.diffuse_red, v.diffuse_green, v.diffuse_blue]);
end
