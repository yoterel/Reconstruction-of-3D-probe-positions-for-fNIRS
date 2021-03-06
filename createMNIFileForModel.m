addpath("plyToPos");

mniModelPath = "C:\Globus\emberson-consortium\VideoRecon\MATLAB\infantModelMNI.mat";
plyFileDir = "C:\Users\Dean\Downloads";
plyFileName = "infantModel.0.ply";
outputFileName = "NewModelMNI2.mat";
plyFilePath = fullfile(plyFileDir, plyFileName);
stickerHSVPath = "C:\TEMP\stickerHSV.txt";
radiusToStickerRatio = 5;
stickerMinGroupSize = 15;
maxStickerHueDiff = 0.15;

fprintf("Reading ply file\n");
% Use the first variant if reading an already processed ply, otherwise use
% the second
%pc = pcread(plyFilePath)
pc = structToPointCloud(plyread(plyFilePath));

% Some of the models contain green points which are also noise, so a
% stricter than usual thershold for counting as sticker candidates (in
% terms of hue) is used here.
load(stickerHSVPath, 'stickerHSV');
candidatesPc = getStickerCandidates(pc, stickerHSV, maxStickerHueDiff); 
candidates = candidatesPc.Location;
fprintf("Found sticker candidate vertices\n");

% Use spherical approximations for the model & cap, use them to scale and translate the sticker
% candidate vertices on the cap (one can scale modelStars instead, as done before in a comment)
% Use the first variant if the model.mni file is for an infant, otherwise
% use the second variant.
load(mniModelPath, 'infantModelMNI');
modelMNI = infantModelMNI;
%load(mniModelPath, 'modelMNI');
modelPoints = [modelMNI.X, modelMNI.Y, modelMNI.Z]; 
[~, modelSphereR, scale, translate] = sphereScaleAndTranslate(candidates, modelPoints);
modelPoints = modelPoints * scale + translate;
fprintf("Used approximating spheres to scale and translate candidate sticker points\n");

plyStars = getClosePointClusterCenters(candidates, modelSphereR / radiusToStickerRatio, ...
    stickerMinGroupSize, false);
fprintf("Calculated approximate sticker positions\n");

expectedNumStars = 9;
numFoundStars = size(plyStars, 1);
if numFoundStars ~= expectedNumStars
    error(...
        "Model isn't accurate enough: an incorrect number of stickers was spotted. Expected: %d, actual: %d", ...
        expectedNumStars, numFoundStars);
end

headAndCapIdxs = (size(modelMNI, 1)-expectedNumStars+1):size(modelMNI, 1); 
modelStars = modelPoints(headAndCapIdxs, :); % Coordinates of stars on model
modelStarLabels = modelMNI.labels(headAndCapIdxs); % Ordered labels of stars on model

[bestRegParams, bestProjStars, bestMate, bestDistance] = ...
    calculateBestRegParams(modelStars, plyStars, modelSphereR);
save("C:\temp\createmniformodel.mat");
%load("C:\temp\createmniformodel.mat", 'bestRegParams', 'bestProjStars', 'bestMate', 'bestDistance');
if bestDistance == inf
    error("No sufficiently accurate regulation parameters could be found");
end

% If successful regulation parameters were found, then bestMate should be
% the same length as capStars, meaning we labeled all existing stars
plyLabels = modelStarLabels(bestMate);
plotAdjustedModelVsCap(plyStars, plyLabels, double(bestProjStars), modelStarLabels);

fprintf("Applying calculated transformation to the rest of the model points\n");
transformedModelPoints = applyRegParams(modelPoints, bestRegParams);
labels = modelMNI.labels;
X = transformedModelPoints(:,1);
Y = transformedModelPoints(:,2);
Z = transformedModelPoints(:,3);
modelMNI = table(labels, X, Y, Z);
save(fullfile(plyFileDir, outputFileName), 'modelMNI');
fprintf("Saved new model MNI file to disk\n");

function [bestRegParams, bestProjStars, bestMate, bestDistance] = calculateBestRegParams(...
    modelStars, plyStars, modelSphereR)
% CALCULATEBESTREGPARAMS Calculates the optimal reg params for adjusting
%   existing model stars to the cap stars. Returns the parameters and the
%   adjusted model stars
%
%   INPUT:
%       existsStars: Array of locations of model stars which exist on the
%                    cap
%          plyStars: Array of locations of cap stars
%      modelSphereR: Radius of the model's approximating sphere
%
%   OUTPUT:
%       bestRegParams: struct returned by absor which approximates the optimal
%           translation and rotation for adjusting the model to the cap
%       bestProjStars: coordinates of the model stars transformed using the
%           optimal reg params
n = size(plyStars, 1);
% Two n * 3 matrices containing all possible triplets in 1:n
capTripletOrder = nchoosek(1:n, 3);
modelTripletOrder = nchoosek(1:n, 3);
bestDistance = inf;
fprintf("Processing triplets\n");
for ii = 1:size(modelTripletOrder,1)
    fprintf("Triplet number %d\n", ii);
    modelTriplet = modelStars(modelTripletOrder(ii,:),:);    
    for kk = 1:size(capTripletOrder,1)
        currTriplet = capTripletOrder(kk,:);
        permTriplet = perms(currTriplet);
        for ll = 1:size(permTriplet,1)
            plyTriplet = plyStars(permTriplet(ll,:),:);
            % regParams is a struct containing the optimal rotation and
            % translation between the two triplets
            regParams = absor(modelTriplet', plyTriplet');
            
            % Transform the model stars and project them on the cap
            projStars = applyRegParams(modelStars, regParams);
            mate = minDistanceMatchPoints(plyStars, projStars, modelSphereR);
            if any(mate < 0) || length(mate) < n
                % The case where the matching isn't complete (should occur
                % only if some points are further apart than the sphere's
                % radius).
                d = inf;
            else
                % The error is the sum of squared distances between pairs
                % of matched points
                d = sum(sum((plyStars - projStars(mate,:)).^2));
            end

            if d < bestDistance
                bestDistance = d;
                bestProjStars = projStars;
                bestRegParams = regParams;
                bestMate = mate;
            end
        end
    end
end
if bestDistance == inf
    % No successful reg parameters were found, assign placeholder values
    % for remaining returned variables
    bestProjStars = [];
    bestRegParams = {};
    bestMate = [];
end
end

function [transformed] = applyRegParams(points, regParams, invert)
% APPLYREGPARAMS transforms the given set of points using the given
% regParams struct (usually returned from absor)
transformationMatrix = regParams.M';
if (nargin > 2 && invert)
    transformationMatrix = transformationMatrix^-1;
end
transformed = [points, ones(size(points, 1), 1)] * transformationMatrix;
transformed = transformed(:, 1:3);
end

function [] = plotAdjustedModelVsCap(plyStars, plyLabels, bestProjStars, modelStarLabels)
% PLOTADJUSTEDMODELVSCAP Plot the cap labels (blue circles) vs. adjusted model labels (red stars) 
figure; hold on; axis equal;
plot3(plyStars(:,1),plyStars(:,2),plyStars(:,3),'ob');
text(plyStars(:,1),plyStars(:,2),plyStars(:,3),plyLabels,'color',[0,0,1]);
plot3(bestProjStars(:,1),bestProjStars(:,2),bestProjStars(:,3),'pr');
text(bestProjStars(:,1),bestProjStars(:,2),bestProjStars(:,3),modelStarLabels,'color',[1,0,0]);
drawnow;
end