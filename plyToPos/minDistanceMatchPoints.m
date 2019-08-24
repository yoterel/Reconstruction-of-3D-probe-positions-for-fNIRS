function [mate] = minDistanceMatchPoints(sourcePoints, targetPoints, modelSphereR)
%MINDISTANCEMATCHPOINTS Returns an array the same size as sourcePoints
%   which uniquely maps each point in sourcePoints to a corresponding point
%   in targetPoints, such that mate(i) is the index of a point in
%   targetPoints mapped to sourcePoints(i), or -1 if the point is unmatched
%   (this should not occur unless targetPoints contains less points than
%   sourcePoints).
%
%   The matching is calculated by attempting to minimize the total
%   euclidean distance between pairs of matched points, using a maximum
%   weight matching algorithm in graphs.
n = size(sourcePoints, 1);
m = size(targetPoints, 1);

% We build a graph with m + n vertices, where edges are only between source
% points and target points, and the weight is inversly proportional to the
% distance between each pair of points.
edges = zeroes(n * m, 3);
numEdges = 0;
for j = 1:n
    for k = 1:m
        d = norm(sourcePoints(j, :) - targetPoints(k, :));
        if (d < modelSphereR)
            numEdges = numEdges + 1;
            edges(numEdges, 1) = j + m;
            edges(numEdges, 2) = k;
            edges(numEdges, 3) = d * d;
        end
    end
end

% remove excess entries in the edges matrix
edges = edges(1:numEdges, :); 

% Flip the weights of the edges to make them inversly proportional to the
% distance, while still keeping them positive.
% The same effect can possibly be achieved by requiring only maximal sized
% matchings and using -1 * the original weight of the edge.
edges(:, 3) = max(edges(:, 3)) - edges(:, 3) + 1;

% TODO: we can probably switch this algorithm with an algorithm that works
% for bipartite graphs, since according to the documentation, in that case
% the problem becomes strictly easier and there are more efficient
% algorithms. Reference: http://jorisvr.nl/article/maximum-matching
mate = maxWeightMatching(edges, false);

% We are only interested in mappings for source vertices
mate = mate(m + 1:end);
end

