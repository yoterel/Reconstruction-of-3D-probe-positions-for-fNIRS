function [subX, subY, subZ] = getCapOnHeadPositions(modelOnCap, bestCapHead, modelHeadIdxs)
% GETCAPONHEADPOSITIONS returns x,y,z coordinates of model points
% where the coordinates of the points on the head in the model were
% replaced with the actual positions of the head relative to the cap
subX = modelOnCap(:,1);
subY = modelOnCap(:,2);
subZ = modelOnCap(:,3);
subX(modelHeadIdxs) = bestCapHead(:, 1);
subY(modelHeadIdxs) = bestCapHead(:, 2);
subZ(modelHeadIdxs) = bestCapHead(:, 3);
end

