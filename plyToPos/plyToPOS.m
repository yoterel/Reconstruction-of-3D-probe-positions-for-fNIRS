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

candidates = getAndPlotStickerCandidatePoints(mesh, stickerHSV);
fprintf("Calculated sticker candidate points\n");

%load(['model',filesep,'modelStars.mat']); %load modelStars, modelLabels, modelSphereC,modelSphereR
load(mniModelPath, 'modelMNI');
%load(mniModelPath, 'infantModelMNI');
%modelMNI = infantModelMNI;
fprintf("Loaded MNI model\n");

% Use spherical approximations for the model & cap, use them to scale and translate the sticker
% candidate vertices on the cap (one can scale modelStars instead, as done before in a comment)
modelPoints = [modelMNI.X, modelMNI.Y, modelMNI.Z]; 
[~, modelSphereR, scale, translate] = sphereScaleAndTranslate(modelPoints, candidates);
candidates = candidates * scale + translate;
fprintf("Used approximating spheres to scale and translate candidate sticker points\n");

% Approximates cap sticker positions using distance based clustering, plot
% clusters
figure; axis equal; hold on;
capStars = getClosePointClusterCenters(candidates, modelSphereR / radiusToStickerRatio, ...
    stickerMinGroupSize, true);
fprintf("Calculated approximate sticker positions\n");

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
[bestRegParams, bestProjStars, bestMate, bestDistance] = ...
    calculateBestRegParams(existStars, capStars, modelSphereR);
if bestDistance == inf
    error("No sufficiently accurate regulation parameters could be found");
end
% If successful regulation parameters were found, then bestMate should be
% the same length as capStars, meaning we labeled all existing stars
capLabels = existLabels(bestMate);
plotAdjustedModelVsCap(capStars, capLabels, bestProjStars, existLabels);

% rotate model to MNI. Use this rotation for the cap
capStars = applyRegParams(capStars, bestRegParams, true);

% reorder capStars and capLabels according to modelLabels
tempCapStars = capStars;
for i = 1:length(existLabels)
    capStars(i,:) = tempCapStars(strcmp(capLabels, existLabels{i}), :);
end
capLabels = existLabels;

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
capHead = capStars(capHeadIdxs,:);
modelHead = modelPoints(modelHeadIdxs,:);
[bestReg, capHead, bestScale] = findHeadTransformation(capHead, modelHead);
capStars = applyRegParams(capStars.*bestScale, bestReg);
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
capCap = capStars(capCapIdxs,:);
[modelReg, ~] = absor(modelCap',capCap');
modelOnCap = applyRegParams(modelPoints, modelReg);

graphResults(modelHead, capHead, modelOnCap, capCap, modelPoints, modelCapIdxs);
[subX, subY, subZ] = getCapOnHeadPositions(modelOnCap, capHead, modelHeadIdxs);
createPOS(outputDir, nirsModelPath, shimadzuFilePath, modelLabels, subX, subY, subZ);
    
close all
end

function [candidates] = getAndPlotStickerCandidatePoints(mesh, stickerHSV)
% GETANDPLOTSTICKERCANDIDATEPOINTS Find coordinates of points in the ply whose hue is close to the
%   given stickers' hue
pc = structToPointCloud(mesh);
candidatesPc = getStickerCandidates(pc, stickerHSV);
candidates = candidatesPc.Location;
fprintf("Found sticker candidate vertices\n");
% Plot the candidates
plot3(candidates(:,1), candidates(:,2), candidates(:,3), '.');
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

function [bestRegParams, bestProjStars, bestMate, bestDistance] = calculateBestRegParams(...
    existStars, capStars, modelSphereR)
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
bestDistance = inf;
fprintf("Processing triplets\n");
% TODO: maybe there's a way to accelerate this part? Use all cores?
totalLoopStart = tic;
minDistanceMatchElapsed = 0;
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
%             modelProjStars = zeros(size(existStars));
%             for mm = 1:size(existStars,1)
%                 modelProjStars(mm,:) = tModelStars(mm,:)*capTriplet;
%             end
%             dists = ones(size(existStars,1),size(modelStars,1));
%             for dd = 1:size(existStars,1)
%                 dists(dd,:) = sqrt(sqrt(sum((existStars(dd,:)-modelProjStars).^2,2)));
%             end
%             [match, cost] = munkres(dists);
            curStart = tic;
            mate = minDistanceMatchPoints(capStars, modelProjStars, modelSphereR);
            minDistanceMatchElapsed = minDistanceMatchElapsed + toc(curStart);
            if any(mate < 0) || length(mate) < n
                % The case where the matching isn't complete (should occur
                % only if some points are further apart than the sphere's
                % radius).
                d = inf;
            else
                % The error is the sum of squared distances between pairs
                % of matched points
                d = sum(sum((capStars - modelProjStars(mate,:)).^2));
            end

            if d < bestDistance
                bestDistance = d;
                bestProjStars = modelProjStars;
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
totalLoopElapsed = toc(totalLoopStart);
fprintf("Total time spent in loop: %f, total time spent in minDistanceMatchPoints: %f\n", ...
    totalLoopElapsed, minDistanceMatchElapsed);
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

function [] = plotAdjustedModelVsCap(capStars, capLabels, bestProjStars, existLabels)
% PLOTADJUSTEDMODELVSCAP Plot the cap labels (blue circles) vs. adjusted model labels (red stars) 
figure; hold on; axis equal;
plot3(capStars(:,1),capStars(:,2),capStars(:,3),'ob')
text(capStars(:,1),capStars(:,2),capStars(:,3),capLabels,'color',[0,0,1])
plot3(bestProjStars(:,1),bestProjStars(:,2),bestProjStars(:,3),'pr')
text(bestProjStars(:,1),bestProjStars(:,2),bestProjStars(:,3),existLabels,'color',[1,0,0])
end

function [bestReg, transformedCapHead, bestScale] = findHeadTransformation(capHead, modelHead)
% FINDOPTIMALSCALING Finds the best scaling, rotation and translation parameters for fitting the
%   current cap head stars to the model stars
% Perform grid search of axis-aligned scaling, using only the head points,
% and realign at each step.
fprintf("Finding optimal scaling parameters\n");
scalespace = logspace(log10(0.5),log10(2),20);
bestD = inf;
for scX = scalespace
    for scY = scalespace
        for scZ = scalespace
            testCapHead = capHead.*[scX,scY,scZ]; % implicit expansion
            testReg = absor(testCapHead', modelHead');
            testCapHead = applyRegParams(testCapHead, testReg);
            testCapHead = testCapHead(:,1:3);
            % TODO: maybe not needed, need to use the labels
            [~,d] = knnsearch(modelHead, testCapHead); 
            %if sum(d) < bestD && (length(idx) == length(unique(idx)) || size(capStars,1) > size(existStars,1))
            %if ( cost < bestD )
            cost = sum(d.^2);
            if cost < bestD
                bestReg = testReg;
                transformedCapHead = testCapHead;
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