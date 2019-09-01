function [existStars, existLabels] = findExistingStarsAndLabels(modelMNI, missingStars)
% FINDEXISTINGSTARSANDLABELS returns ordered arrays of the stars and labels
%   in the model that are in the vid ply.

% The last 9 points in the model MNI are the relevant ones (TODO: be more
% precise here, the points is that headAndCapIdx are the last 9 indices in modelMNI)
headAndCapIdxs = size(modelMNI, 1)-8:size(modelMNI, 1); 
modelStars = table2array(modelMNI(headAndCapIdxs, 2:4)); % Coordinates of stars on model
modelStarLabels = modelMNI.labels(headAndCapIdxs); % Ordered labels of stars on model

existStarsBitVec = ~ismember(modelStarLabels, missingStars);
existStars = modelStars(existStarsBitVec, :);
%existStars = existStars - mean(existStars);
existLabels = modelStarLabels(existStarsBitVec);
end
