function [transformedPc, sourceC, sourceR] = sphereScaleAndTranslate(sourcePc, targetPc)
%SPHERESCALEANDTRANSLATE Scales and translates a target point cloud to
%match a source point cloud, based on spherical models of the two.
%   INPUT:
%       sourcePc: The source point cloud (n * 3 matrix)
%       targetPc: The target point cloud (n * 3 matrix)
%
%   OUTPUT:
%       transformedPc: A scaled + translated version of the target point
%                      cloud
%             sourceC: The center of the sphere approximating the source
%                      point cloud
%             sourceR: The radius of the sphere approximating the source
%                      point cloud
[sourceC, sourceR] = sphereFit(sourcePc);
[targetC, targetR] = sphereFit(targetPc);
relativeR = sourceR / targetR;
relativeC = sourceC - targetC;
transformedPc = targetPc * relativeR + relativeC;
end