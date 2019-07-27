function [mate] = maxWeightMatchingHelper(capStars, projStars, modelSphereR)
%MAXWEIGHTMATCHINGHELPER TODO: add proper documentation. Summary of this function goes here
%   Detailed explanation goes here
% TODO: preallocate in right size?
edges = [];
mm = 1;
for nn = 1:size(capStars,1)
    for pp = 1:size(projStars, 1)
        d = norm(capStars(nn, :) - projStars(pp, :));
        if (d < modelSphereR)
            edges(mm, 1) = nn + size(projStars, 1);
            edges(mm, 2) = pp;
            edges(mm, 3) = d * d;
            mm = mm + 1;
        end
    end
end
edges(:,3) = max(edges(:,3)) - edges(:,3) + 1;
mate = maxWeightMatching(edges, false);
mate = mate(size(projStars,1)+1:end);
end

