function [] = plyToPOS(plyFilePath, stickerHSV, mniModelPath, shimadzuFilePath, outputDir, ...
    nirsModelPath, stickerMinGroupSize, radiusToStickerRatio)
% PLYTOPOS converts a ply file to a POS
%            plyFilePath: Path to the ply file to convert
%             stickerHSV: A normalized (between 0 and 1) 1ª3 hsv representation of the
%                         stickers' color
%           mniModelPath: Path of a file containing info on the model of the cap itself
%                         (labels and 3d positions of points)
%       shimadzuFilePath: Path of the shimadzu output file
%              outputDir: The output dir for the current script
%          nirsModelPath: 
%    stickerMinGroupSize: Minimum number of points in a group for it to be
%                         considered a separate sticker.
%   radiusToStickerRatio: Relative distance for which 2 points are considered the same sticker

fprintf("Reading ply file\n");
mesh = plyread(plyFilePath);
fprintf("Finished reading ply file, finding sticker candidate points\n");

vertices = [mesh.vertex.x mesh.vertex.y mesh.vertex.z];
candidates = getAndPlotStickerCandidatePoints(mesh, vertices, stickerHSV);
fprintf("Calculated sticker candidate points\n");

%load(['model',filesep,'modelStars.mat']); %load modelStars, modelLabels, modelSphereC,modelSphereR
load(mniModelPath, 'modelMNI');
%load(mniModelPath, 'infantModelMNI');
%modelMNI = infantModelMNI;
fprintf("Loaded MNI model\n");

% Use spherical approximations for the model & cap, use them to scale and translate the sticker
% candidate vertices on the cap (one can scale modelStars instead, as done before in a comment)
modelPoints = [modelMNI.X, modelMNI.Y, modelMNI.Z]; 
[candidates, ~, modelSphereR] = sphereScaleAndTranslate(modelPoints, candidates);
fprintf("Used approximating spheres to scale and translate candidate sticker points\n");
capStars = calculateCapStickerPositions(candidates, modelSphereR, radiusToStickerRatio, ...
    stickerMinGroupSize);

%% Add the manually added points
%capStars = [capStars; [-7.888 3.4265 -2.053]*modelSphereR/capSphereR+modelSphereC-capSphereC];
%manualPoints = [-4.561 0.562 3.81]; %subject 2
%manualPoints = [-1.246 1.953 -1.643; -0.893 1.65 -1.731]; %2783a
manualPoints = [];
if ~isempty(manualPoints)
    capStars = [capStars;manualPoints * relativeR + relativeC];
end

% TODO: this is probably figure 2. Add description.
%capStars = capStars - mean(capStars);% implicit expansion
plot3(capStars(:,1), capStars(:,2), capStars(:,3), 'p', 'markersize', 25, 'MarkerFaceColor', 'k');

%% Pick sticker + model point triplets, try to find best match, get bestRegParams
% TODO: why are we using missing stars? Aren't they supplied by the user?
% missingStars = {'Nz';'Iz';'AR';'AL'};
missingStars = {}; % TODO: how do we use this? With manual points? Are these missing stars on the model?

% The last 9 points in the model MNI are the relevant ones (TODO: be more
% precise here, the points is that headAndCapIdx are the last 9 indices in modelMNI)
headAndCapIdxs = size(modelMNI, 1)-8:size(modelMNI, 1); 
modelStars = table2array(modelMNI(headAndCapIdxs, 2:4)); % Coordinates of stars on model
modelStarLabels = modelMNI.labels(headAndCapIdxs); % Ordered labels of stars on model
[existStars, existLabels] = findExistingStarsAndLabels(modelStarLabels, modelStars, missingStars);
[bestRegParams, bestProjStars] = calculateBestRegParams(existStars, capStars, modelSphereR);
[capStars, capLabels] = labelCapStars(capStars, bestProjStars, modelSphereR, existLabels);
plotAdjustedModelVsCap(capStars, capLabels, bestProjStars, existLabels);

% rotate model to MNI. Use this rotation for the cap
capStars = applyRegParams(capStars, bestRegParams, true);
capStars = capStars(:,1:3);

% reorder capStars and capLabels according to modelLabels
tempCapStars = capStars;
for i = 1:length(existLabels)
    capStars(i,:) = tempCapStars(strcmp(capLabels, existLabels{i}), :);
end
capLabels = existLabels;

%% perform grid search of axis-aligned scaling, and realign at each step.
% USING ONLY THE HEAD POINTS
% TODO: use consts for point labels?
modelLabels = modelMNI.labels;
fprintf("Performing grid search of axis-aligned scaling\n");
headLabelNames = {'Nz', 'Cz', 'AR', 'AL'};
capHeadIdxs = ismember(capLabels, headLabelNames);
existPointsBitVec = ~ismember(modelLabels, missingStars);
modelHeadIdxs = ismember(modelLabels, headLabelNames) & existPointsBitVec;
capLabelNames = {'Front', 'Cz', 'Right', 'Left', 'Pz', 'Iz'};
capCapIdxs = ismember(capLabels, capLabelNames);
modelCapIdxs = ismember(modelLabels, capLabelNames) & existPointsBitVec;

modelHead = modelPoints(modelHeadIdxs,:);
[bestReg, bestCapHead, bestScale] = findOptimalScaling(capStars, capHeadIdxs, modelHead);

% rotate + scale model to the best aligned result.
fprintf("Perfroming rotations & scaling\n");
scaledCap = applyRegParams(capStars.*bestScale, bestReg);
scaledCap = scaledCap(:,1:3);
%scaledModel = [modelPoints.*bestScale ones(size(modelPoints,1),1)]*bestReg.M';

% rotate scaled model to match the best to the CAP POINTS ONLY.
modelCap = modelPoints(modelCapIdxs,:);
modelStarsToErase = true(length(modelCap(:,1)),1);
capCapLabels = capLabels(capCapIdxs);
modelCapLabels = modelLabels(modelCapIdxs);
for i = 1:size(modelCapLabels,1)
    if any(ismember(capCapLabels,modelCapLabels{i}))
        modelStarsToErase(i) = false;
    end
end
modelCap(modelStarsToErase,:) = [];
capCap = scaledCap(capCapIdxs,:);
[modelReg, ~] = absor(modelCap',capCap');
modelOnCapCap = [modelPoints ones(size(modelPoints,1),1)]*modelReg.M';
modelOnCapCap(:,4) = [];

graphResults(modelHead, bestCapHead, modelOnCapCap, capCap, modelPoints, modelCapIdxs);
% use fiber points from resulting model to estimate positions.

createPOS(outputDir, nirsModelPath, shimadzuFilePath, modelLabels, modelOnCapCap, ...
    modelHeadIdxs, bestCapHead);
    
close all
end

function [candidates] = getAndPlotStickerCandidatePoints(mesh, vertices, stickerHSV)
% GETANDPLOTSTICKERCANDIDATEPOINTS Find coordinates of points in the ply whose hue is close to the
%   given stickers' hue
verColors = [mesh.vertex.diffuse_red mesh.vertex.diffuse_green mesh.vertex.diffuse_blue];
verColorsHSV = rgb2hsv(double(verColors)/255);
trgHue = stickerHSV(1);
candidates = vertices(abs(verColorsHSV(:,1)-trgHue) < 0.15 & ... 
    ((verColorsHSV(:,2) > 0.2 & verColorsHSV(:,3) > 0.2) | (verColorsHSV(:,2) > 0.3 & verColorsHSV(:,3) > 0.1)), :);
fprintf("Found sticker candidate vertices\n");
% Plot the candidates
plot3(candidates(:,1), candidates(:,2), candidates(:,3), '.');
end

function [labels] = divideStickerCandidatesIntoClouds(candidates, modelSphereR, radiusToStickerRatio)
% DIVIDESTICKERCANDIDATESINTOCLOUD Groups the sticker candidate points into
%   point clouds, and returns a label column vector specifying the cloud id for
%   each sticker
distMatrix = zeros(length(candidates));
for v = 1:length(candidates)
    distMatrix(:,v) = sqrt(sum([candidates(:,1)-candidates(v,1) candidates(:,2)-candidates(v,2) candidates(:,3)-candidates(v,3)].^2,2));
end
% Initialize a binary matrix representing a graph where each two points
% close enough to be on the same sticker are connected
adjMatrix = zeros(length(candidates));
maxStickerLength = modelSphereR / radiusToStickerRatio;
adjMatrix(distMatrix < maxStickerLength) = 1;
% Labels is a column vector containing a connected component id for each candidate vertex
[labels, ~] = graphConnectedComponents(adjMatrix);
end

function [capStars] = calculateCapStickerPositions(candidates, modelSphereR, ...
    radiusToStickerRatio, stickerMinGroupSize)
% CALCULATECAPSTICKERPOSITIONS Approximates cap sticker positions using the
%   candidate sticker points
labels = divideStickerCandidatesIntoClouds(candidates, modelSphereR, radiusToStickerRatio);
capStars = convertPointCloudsIntoPositions(candidates, labels, stickerMinGroupSize);
fprintf("Calculated approximate sticker positions\n");
end

function [capStars] = convertPointCloudsIntoPositions(candidates, labels, stickerMinGroupSize)
% CONVERTPOINTCLOUDSINTOPOSITIONS Converts the grouped candidate points
%   into individual sticker positions
figure; axis equal; hold on
numConnectedComponents = max(labels);
counter = zeros(numConnectedComponents, 1);
mids = zeros(numConnectedComponents, 3);
for i = 1:numConnectedComponents
    relevantCandidates = candidates(labels == i,:);
    scatter3(relevantCandidates(:,1), relevantCandidates(:,2), relevantCandidates(:,3));
    counter(i) = size(relevantCandidates, 1);
    mids(i,:) = sum(relevantCandidates, 1)./counter(i);
end
% Throw away stickers that are too small
capStars = mids(counter > stickerMinGroupSize,:); 
end

function [existStars, existLabels] = findExistingStarsAndLabels(modelStarLabels, modelStars, ...
    missingStars)
% FINDEXISTINGSTARSANDLABELS returns ordered arrays of the stars and labels
%   in the model that are in the vide ply.
existStarsBitVec = ~ismember(modelStarLabels, missingStars);
existStars = modelStars(existStarsBitVec, :);
%modelTriplet = modelTriplet - mean(existStars);
%existStars = existStars - mean(existStars);
existLabels = modelStarLabels(existStarsBitVec);
end

function [bestRegParams, bestProjStars] = calculateBestRegParams(existStars, capStars, ...
    modelSphereR)
% CALCULATEBESTREGPARAMS Calculates the optimal reg params for adjusting
%   existing model stars to the cap stars. Returns the parameters and the
%   adjusted model stars
%
%   INPUT:
%       existsStars: Array of locations of model stars which exist on the
%                    cap
%          capStars: Array of locations of cap stars
%      modelSphereR: Radius of the model's approximating sphere
%
%   OUTPUT:
%       bestRegParams: struct returned by absor which approximates the optimal
%           translation and rotation for adjusting the model to the cap
%       bestProjStars: coordinates of the model stars transformed using the
%           optimal reg params
% TODO: validate that existStars and capStars are the same length?
n = size(capStars, 1);
m = size(existStars, 1);
% Two n * 3 matrices containing all possible triplets in 1:n, 1:m
capTripletOrder = nchoosek(1:n, 3);
modelTripletOrder = nchoosek(1:m, 3);
bestD = inf;
fprintf("Processing triplets\n");
% TODO: maybe there's a way to accelerate this part? Use all cores?
for ii = 1:size(modelTripletOrder,1)
    fprintf("Triplet number %d\n", ii);
    modelTriplet = existStars(modelTripletOrder(ii,:),:);    
    for kk = 1:size(capTripletOrder,1)
        currTriplet = capTripletOrder(kk,:);
        permTriplet = perms(currTriplet);
        for ll = 1:size(permTriplet,1)
            capTriplet = capStars(permTriplet(ll,:),:);
            % regParams is a struct containing the optimal rotation and
            % translation between the two triplets
            regParams = absor(modelTriplet',capTriplet');
            
            % TODO: Make sure I understand this part. Maybe the double
            % transformations are unnecessary (we transform both the model
            % and late on the cap).
            % Transform the model stars and project them on the cap
            modelProjStars = applyRegParams(existStars, regParams);
            modelProjStars = modelProjStars(:,1:3);
%             modelProjStars = zeros(size(existStars));
%             for mm = 1:size(existStars,1)
%                 modelProjStars(mm,:) = tModelStars(mm,:)*capTriplet;
%             end
%             dists = ones(size(existStars,1),size(modelStars,1));
%             for dd = 1:size(existStars,1)
%                 dists(dd,:) = sqrt(sqrt(sum((existStars(dd,:)-modelProjStars).^2,2)));
%             end
%             [match, cost] = munkres(dists);
            
            mate = minDistanceMatchPoints(capStars, modelProjStars, modelSphereR);
            if any(mate < 0) || length(mate) < n
                % The case where the matching isn't complete (should occur
                % only if some points are further apart than the sphere's
                % radius).
                d = inf;
            else
                d = sum(sum((capStars - modelProjStars(mate,:)).^2,2));
            end

            if d < bestD
                bestD = d;
                bestProjStars = modelProjStars;
                bestRegParams = regParams;
            end
        end
    end
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
end

function [capStars, capLabels] = labelCapStars(capStars, bestProjStars, modelSphereR, existLabels)
% LABELCAPSTARS Uses bestProjStars to label the cap stars
fprintf("Applying the calculated regulation parameters\n");
mate = minDistanceMatchPoints(capStars, bestProjStars, modelSphereR);
% TODO: is it possible that there are unmatched entries? In such case the
% error shold be inf and we should probably just stop the script. In
% general, this function can probably be deleted and we can just return the
% mate we found in calculateBestRegParams
capStars(mate < 1,:) = [];
mate(mate < 1) = [];
capLabels = existLabels(mate);
end

function [] = plotAdjustedModelVsCap(capStars, capLabels, bestProjStars, existLabels)
% PLOTADJUSTEDMODELVSCAP Plot the cap labels (blue circles) vs. adjusted model labels (red stars) 
figure; hold on; axis equal;
plot3(capStars(:,1),capStars(:,2),capStars(:,3),'ob')
text(capStars(:,1),capStars(:,2),capStars(:,3),capLabels,'color',[0,0,1])
plot3(bestProjStars(:,1),bestProjStars(:,2),bestProjStars(:,3),'pr')
text(bestProjStars(:,1),bestProjStars(:,2),bestProjStars(:,3),existLabels,'color',[1,0,0])
end

function [bestReg, bestCapHead, bestScale] = findOptimalScaling(capStars, capHeadIdxs, modelHead)
% FINDOPTIMALSCALING Finds the best scaling parameters for fitting the
%   current cap stars to the model head
fprintf("Finding optimal scaling parameters\n");
capHead = capStars(capHeadIdxs,:);
scalespace = logspace(log10(0.5),log10(2),20);
bestD = inf;
for scX = scalespace
    for scY = scalespace
        for scZ = scalespace
            testStars = capHead.*[scX,scY,scZ]; % implicit expansion
            testReg = absor(testStars', modelHead');
            testStars = [testStars ones(size(testStars,1),1)]*testReg.M';
            testStars = testStars(:,1:3);
            %%%% maybe not needed, need to use the labels
            [~,d] = knnsearch(modelHead,testStars); 
            %if sum(d) < bestD && (length(idx) == length(unique(idx)) || size(capStars,1) > size(existStars,1))
            %if ( cost < bestD )
            cost = sqrt(sum(d.^2));
            if cost < bestD
                bestReg = testReg;
                bestCapHead = testStars;
                bestScale = [scX,scY,scZ];
                bestD = cost;
            end
        end
    end
end
end

% TODO: this is the graph that looks like a point cloud of the entire head.
% Figure out what it means exactly and rename function
function [] = graphResults(modelHead, bestCapHead, modelOnCapCap, capCap, modelPoints, ...
    modelCapIdxs)
% GRAPHRESULTS
figure; hold on; axis equal; 
scatter3(modelHead(:,1),modelHead(:,2),modelHead(:,3));
scatter3(bestCapHead(:,1),bestCapHead(:,2),bestCapHead(:,3))
scatter3(modelOnCapCap(:,1),modelOnCapCap(:,2),modelOnCapCap(:,3))
scatter3(capCap(:,1),capCap(:,2),capCap(:,3))
scatter3(modelPoints(modelCapIdxs,1),modelPoints(modelCapIdxs,2),modelPoints(modelCapIdxs,3))
end