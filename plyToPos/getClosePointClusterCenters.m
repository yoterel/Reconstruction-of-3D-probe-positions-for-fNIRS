function centers = getClosePointClusterCenters(points, maxDistance, minClusterSize, plotClusters)
%GETCLOSEPOINTCLUSTERCENTERS Clusters the given points based on the
%distance between them, such that each two points in each cluster can't be
%further apart then the maximum given distance, and returns the centers of
%the clusters. plotClusters is a boolean specifying whether to plot the
%different clusters or not
labels = clusterClosePoints(points, maxDistance);
centers = convertPointCloudsIntoPositions(points, labels, minClusterSize, plotClusters);
end

function labels = clusterClosePoints(points, maxDistance)
%CLUSTERCLOSEPOINTS Returns a vector assigning each given point to a
%cluster of other close points from the given set of points. The clusters
%are determined by the given maximum distance.
distMatrix = zeros(length(points));
for v = 1:length(points)
    distMatrix(:,v) = sqrt(sum((points - points(v,:)).^2,2));
end

% Initialize a binary matrix representing a graph where each two points
% close enough to be on the same sticker are connected
adjMatrix = zeros(length(points));
adjMatrix(distMatrix < maxDistance) = 1;

% Labels is a column vector containing a connected component id for each candidate vertex
[labels, ~] = graphConnectedComponents(adjMatrix);
end

function centers = convertPointCloudsIntoPositions(points, labels, minClusterSize, plotClusters)
% CONVERTPOINTCLOUDSINTOPOSITIONS Converts the grouped candidate points
%   into individual sticker positions
numConnectedComponents = max(labels);
counter = zeros(numConnectedComponents, 1);
mids = zeros(numConnectedComponents, 3);
if plotClusters
    hold on;
end
for i = 1:numConnectedComponents
    relevantPoints = points(labels == i,:);
    if plotClusters
        scatter3(relevantPoints(:,1), relevantPoints(:,2), relevantPoints(:,3));
    end
    counter(i) = size(relevantPoints, 1);
    mids(i,:) = sum(relevantPoints, 1)./counter(i);
end

% Throw away clusters that are too small
centers = mids(counter > minClusterSize, :); 
end