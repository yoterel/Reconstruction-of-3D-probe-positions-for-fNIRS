function [pc] = structToPointCloud(pcStruct)
%STRUCTTOPOINTCLOUD Converts a struct returned from plyread into a
%pointCloud object
v = pcStruct.vertex;
colors = [v.diffuse_red, v.diffuse_green, v.diffuse_blue];
normals = [v.nx, v.ny, v.nz];
pc = pointCloud([v.x, v.y, v.z], 'Color', colors/255, 'Normal', normals);
end
