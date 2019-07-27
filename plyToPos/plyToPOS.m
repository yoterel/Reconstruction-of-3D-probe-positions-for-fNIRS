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

mesh = plyread(plyFilePath);
fprintf("Finished reading ply file\n");

vertices = [mesh.vertex.x mesh.vertex.y mesh.vertex.z];
candidates = getAndPlotStickerCandidatePoints(mesh, vertices, stickerHSV);

[capSphereC, capSphereR] = sphereFit(candidates);
fprintf("Performed sphere fit on candidates 1\n");

%load(['model',filesep,'modelStars.mat']); %load modelStars, modelLabels, modelSphereC,modelSphereR
load(mniModelPath, 'modelMNI');
%load(mniModelPath, 'infantModelMNI');
%modelMNI = infantModelMNI;
fprintf("Loaded MNI model\n");

% Find an approximating sphere for the model, use it to scale and translate the sticker candidate 
% vertices on the cap (one can also scale modelStars instead, as was done before in a comment)
[modelSphereC, modelSphereR] = sphereFit([modelMNI.X,modelMNI.Y,modelMNI.Z]);
fprintf("Performed sphere fit on model mni\n")
candidates = candidates * modelSphereR / capSphereR+modelSphereC - capSphereC;

capStars = calculateCapStickerPositions(candidates, modelSphereR, radiusToStickerRatio, ...
    stickerMinGroupSize);

% The last 9 points in the model MNI are the relevant ones (TODO: be more
% precise here, the points is that headAndCapIdx are the last 9 indices in modelMNI)
headAndCapIdxs = size(modelMNI, 1)-8:size(modelMNI, 1); 
modelStars = table2array(modelMNI(headAndCapIdxs, 2:4));
modelStarLabels = modelMNI.labels(headAndCapIdxs);

%% Add the manually added points
%capStars = [capStars; [-7.888 3.4265 -2.053]*modelSphereR/capSphereR+modelSphereC-capSphereC];
%manualPoints = [-4.561 0.562 3.81]; %subject 2
%manualPoints = [-1.246 1.953 -1.643; -0.893 1.65 -1.731]; %2783a
manualPoints = [];
if ~isempty(manualPoints)
    capStars = [capStars; manualPoints*modelSphereR/capSphereR+modelSphereC-capSphereC];
end

% TODO: this is probably figure 2. Add description.
%capStars = capStars - mean(capStars);% implicit expansion
plot3(capStars(:,1), capStars(:,2), capStars(:,3), 'p', 'markersize', 25, 'MarkerFaceColor', 'k');

%% save for model. Order of labels change from run to run.
% close all
% modelStars = mids(counter > groupSize,:);
% modelLabels = {'Fz';'Cz';'Nz';'Iz';'Right';'Pz';'AL';'Left';'AR'};
% [modelSphereC,modelSphereR] = sphereFit(candidates)
% [x,y,z] = sphere(10);
% sphereCoords = [x(:),y(:),z(:)];
% sphereCoords = sphereCoords*modelSphereR+modelSphereC;
% figure; axis equal; hold on; 
% plot3(modelStars(:,1),modelStars(:,2),modelStars(:,3),'p','markersize',25,'MarkerFaceColor','k');
% scatter3(sphereCoords(:,1),sphereCoords(:,2),sphereCoords(:,3))
% scatter3(vertices(:,1),vertices(:,2),vertices(:,3),1)
% save([plyPath,'modelStars.mat'],'modelStars','modelLabels','modelSphereC','modelSphereR')

%% Pick sticker + model point triplets, try to find best match, get bestRegParams
% TODO: why are we using missing stars? Aren't they supplied by the user?
% missingStars = {'Nz';'Iz';'AR';'AL'};
missingStars = {}; % TODO: how do we use this? With manual points? Are these missing stars on the model?

% modelTriplet = [modelStars(strcmpi(modelLabels,'front'),:);modelStars(strcmpi(modelLabels,'right'),:);modelStars(strcmpi(modelLabels,'left'),:)];

existStarsBitVec = ~ismember(modelStarLabels, missingStars);
existStars = modelStars(existStarsBitVec, :);
%modelTriplet = modelTriplet - mean(existStars);
%existStars = existStars - mean(existStars);
existLabels = modelStarLabels(existStarsBitVec);

[bestRegParams, bestProjStars] = calculateBestRegParams(existStars, capStars, ...
    modelSphereR);

% Use bestProjStars to label the cap stars
fprintf("Applying the calculated regulation parameters\n");
mate = maxWeightMatchingHelper(capStars, bestProjStars, modelSphereR);
capStars(mate < 1,:) = [];
mate(mate < 1) = [];
capLabels = existLabels(mate);

% Plot the cap labels (blue circles) vs. adjusted model stars (red stars) 
figure; hold on; axis equal;
plot3(capStars(:,1),capStars(:,2),capStars(:,3),'ob')
text(capStars(:,1),capStars(:,2),capStars(:,3),capLabels,'color',[0,0,1])
plot3(bestProjStars(:,1),bestProjStars(:,2),bestProjStars(:,3),'pr')
text(bestProjStars(:,1),bestProjStars(:,2),bestProjStars(:,3),existLabels,'color',[1,0,0])

% rotate model to MNI. Use this rotation for the cap
capStars = [capStars ones(length(capStars), 1)] * (bestRegParams.M^-1)';
capStars = capStars(:,1:3);
modelPoints = [modelMNI.X, modelMNI.Y, modelMNI.Z]; 
modelLabels = [modelMNI.labels];

% reorder capStars and capLabels according to modelLabels
tempCapStars = capStars;
for ii = 1:length(existLabels)
    capStars(ii,:) = tempCapStars(strcmp(capLabels, existLabels{ii}), :);
end
capLabels = existLabels;

%% perform grid search of axis-aligned scaling, and realign at each step.
% USING ONLY THE HEAD POINTS
% TODO: use consts for point labels?
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
scaledCap = [capStars.*bestScale ones(size(capStars,1),1)]*bestReg.M';
scaledCap = scaledCap(:,1:3);
%scaledModel = [modelPoints.*bestScale ones(size(modelPoints,1),1)]*bestReg.M';

% rotate scaled model to match the best to the CAP POINTS ONLY.
modelCap = modelPoints(modelCapIdxs,:);
modelStarsToErase = true(length(modelCap(:,1)),1);
capCapLabels = capLabels(capCapIdxs);
modelCapLabels = modelLabels(modelCapIdxs);
for ii = 1:size(modelCapLabels,1)
    if any(ismember(capCapLabels,modelCapLabels{ii}))
        modelStarsToErase(ii) = false;
    end
end
modelCap(modelStarsToErase,:) = [];
capCap = scaledCap(capCapIdxs,:);
[modelReg, ~] = absor(modelCap',capCap');
modelOnCapCap = [modelPoints ones(size(modelPoints,1),1)]*modelReg.M';
modelOnCapCap(:,4) = [];

%% Graph results
% TODO: this is the graph that looks like a point cloud of the entire head.
% Figure out what it means exactly.
figure; hold on; axis equal; 
scatter3(modelHead(:,1),modelHead(:,2),modelHead(:,3));
scatter3(bestCapHead(:,1),bestCapHead(:,2),bestCapHead(:,3))
scatter3(modelOnCapCap(:,1),modelOnCapCap(:,2),modelOnCapCap(:,3))
scatter3(capCap(:,1),capCap(:,2),capCap(:,3))
scatter3(modelPoints(modelCapIdxs,1),modelPoints(modelCapIdxs,2),modelPoints(modelCapIdxs,3))
% use fiber points from resulting model to estimate positions.

%% create files for spm_fnirs and for Homer2
fprintf("Creating files for spm_fnirs\n");
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

[Ch,Source,Detector] = importChCfgFromShimadzuTXTfile(shimadzuFilePath);
nChannels = length(Ch);
nOptodes = length(unique(Source))+length(unique(Detector));
subX = modelOnCapCap(:,1);
subY = modelOnCapCap(:,2);
subZ = modelOnCapCap(:,3);
subName = modelMNI.labels;

subX(modelHeadIdxs) = bestCapHead(:,1);
subY(modelHeadIdxs) = bestCapHead(:,2);
subZ(modelHeadIdxs) = bestCapHead(:,3);

refNames = {'nz';'ar';'al';'cz';'iz'};
fid = fopen(fullfile(outputDir, "digits.txt"), 'w');
for ref = 1:length(refNames)
    ind = find(strcmpi(subName,refNames{ref}));
    fprintf(fid, '%s: %f %f %f\n', refNames{ref},subX(ind),subY(ind),subZ(ind));
end

SD.Lambda = [780;805;830];
SD.nSrcs = length(unique(Source));
SD.nDets = length(unique(Detector));
SD.SrcPos = zeros(SD.nSrcs,3);
SD.DetPos = zeros(SD.nDets,3);
for src = 1:SD.nSrcs
    ind = find(strcmp(subName,['R',num2str(src)]));
    SD.SrcPos(src,:) = [subX(ind),subY(ind),subZ(ind)];
    fprintf(fid, 's%i: %f %f %f\n', src, subX(ind), subY(ind), subZ(ind));
end
for det = 1:SD.nDets
    ind = find(strcmp(subName,['T',num2str(det)]));
    SD.DetPos(det,:) = [subX(ind),subY(ind),subZ(ind)];
    fprintf(fid, 'd%i: %f %f %f\n', src, subX(ind), subY(ind), subZ(ind));
end
fclose(fid);
SD.MeasList = [repmat([Source,Detector],3,1),ones(nChannels*3,1),[ones(nChannels,1);2*ones(nChannels,1);3*ones(nChannels,1)]];
SD.MeasListAct = ones(size(SD.MeasList,1),1);
SD.SpatialUnit = 'mm';
sdFilePath = fullfile(outputDir, "SD.SD");
save(sdFilePath, 'SD', '-mat')
nirsOutputFilePath = fullfile(outputDir, "nirs_model.nirs");
Shimadzu2nirsSingleFile(shimadzuFilePath, sdFilePath, nirsOutputFilePath);
txt2nirs(shimadzuFilePath);

%% export optode position
fprintf("Exporting optode position\n");
Optode = cell(nOptodes,1);
X = nan(nOptodes,1);
Y = nan(nOptodes,1);
Z = nan(nOptodes,1);

for optInd = 1:(nOptodes/2)
    Optode{optInd} = ['S',num2str(optInd)];
    Optode{optInd + nOptodes/2} = ['D',num2str(optInd)];
    X(optInd) = subX(strcmp(subName,['T',num2str(optInd)]));
    X(optInd + nOptodes/2) = subX(strcmp(subName,['R',num2str(optInd)]));
    Y(optInd) = subY(strcmp(subName,['T',num2str(optInd)]));
    Y(optInd + nOptodes/2) = subY(strcmp(subName,['R',num2str(optInd)]));
    Z(optInd) = subZ(strcmp(subName,['T',num2str(optInd)]));
    Z(optInd + nOptodes/2) = subZ(strcmp(subName,['R',num2str(optInd)]));
end
outputTable = table(Optode,X,Y,Z);
optodePositionsFilePath = fullfile(outputDir, "optode_positions.csv");
writetable(outputTable, optodePositionsFilePath);

%% export reference position
fprintf("Exporting reference position\n");
Reference = {'NzHS';'IzHS';'ARHS';'ALHS';'Fp1HS';'Fp2HS';'FzHS';'F3HS';...
    'F4HS';'F7HS';'F8HS';'CzHS';'C3HS';'C4HS';'T3HS';'T4HS';'PzHS';'P3HS';...
    'P4HS';'T5HS';'T6HS';'O1HS';'O2HS'};
X = nan(length(Reference),1);
Y = nan(length(Reference),1);
Z = nan(length(Reference),1);
refNamesSpmfnirs = {'NzHS';'IzHS';'ARHS';'ALHS';'CzHS'};
refNamesFastrak = {'Nz';'Iz';'AR';'AL';'Cz'};
for refInd = 1:length(refNamesFastrak)
    X(strcmpi(Reference,refNamesSpmfnirs{refInd})) = subX(strcmpi(subName,refNamesFastrak{refInd}));
    Y(strcmpi(Reference,refNamesSpmfnirs{refInd})) = subY(strcmpi(subName,refNamesFastrak{refInd}));
    Z(strcmpi(Reference,refNamesSpmfnirs{refInd})) = subZ(strcmpi(subName,refNamesFastrak{refInd}));
end
outputTable = table(Reference,X,Y,Z);
referencePositionFilePath = fullfile(outputDir, "reference_position.csv");
writetable(outputTable, referencePositionFilePath);

outputTable = table(Ch,Source,Detector);
channelConfigOutputFilePath = fullfile(outputDir, "channel_config.csv");
writetable(outputTable, channelConfigOutputFilePath);

save(fullfile(outputDir, "plyToPOSoutput.mat"));

%% run spm_fnirs spatial tool
fprintf("Running spm_fnirs");
F{1,1}(1,:) = referencePositionFilePath;
F{1,1}(2,:) = optodePositionsFilePath;
F{1,1}(3,:) = channelConfigOutputFilePath;
F{2,1} = nirsModelPath;
spm_fnirs_spatialpreproc_ui(F);
    
close all
end

function [candidates] = getAndPlotStickerCandidatePoints(mesh, vertices, stickerHSV)
% getAndPlotStickerCandidatePoints Find coordinates of points in the ply whose hue is close to the
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
% CALCULATECAPSTARPOSITIONS Approximates cap sticker positions using the
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

function [bestRegParams, bestProjStars] = calculateBestRegParams(existStars, capStars, ...
    modelSphereR)
% CALCULATEBESTREGPARAMS Calculates the optimal reg params for adjusting
%   existing model stars to the cap stars. Returns the parameters and the
%   adjusted model stars

% capFront = capStars(4,:);
% capLeft = capStars(3,:);
% capRight = capStars(5,:);
% testCapTriplet = [capFront;capRight;capLeft];

capTripletOrder = nchoosek(1:size(capStars,1),3);
% capTripletOrder = [4,3,5];

modelTripletOrder = nchoosek(1:size(existStars,1),3);
bestD = inf;
fprintf("Processing triplets\n");
% TODO: maybe there's a way to accelerate this part? Use all cores?
for ii = 1:size(modelTripletOrder,1)
    fprintf("Triplet number %d\n", ii);
    modelTriplet = existStars(modelTripletOrder(ii,:),:);
%     tModelStars = zeros(size(existStars));
%     for jj = 1:size(existStars,1)
%         A = [modelTriplet w*eye(3)];
% %         %tModelStars(jj,:) = existStars(jj,:)/modelTriplet;
% tModelStars(jj,:) = A'\[existStars(jj,:) zeros(1,3)]';
% %         tModelStars(jj,:) = [existStars(jj,:) zeros(1,3)]*A'*(A*A')^-1;
%     end
    
    for kk = 1:size(capTripletOrder,1)
        currTriplet = capTripletOrder(kk,:);
        permTriplet = perms(currTriplet);
        for ll = 1:size(permTriplet,1)
            capTriplet = capStars(permTriplet(ll,:),:);
            regParams = absor(modelTriplet',capTriplet');
            %tform = fitgeotrans(modelTriplet,capTriplet,'nonreflective similarity');
            modelProjStars = [existStars ones(size(existStars,1),1)]*regParams.M';
            modelProjStars = modelProjStars(:,1:3);
            %             modelProjStars = zeros(size(existStars));
            %             for mm = 1:size(existStars,1)
            %                 modelProjStars(mm,:) = tModelStars(mm,:)*capTriplet;
            %             end
%             dists = ones(size(existStars,1),size(modelStars,1));
%             for dd = 1:size(existStars,1)
%                 dists(dd,:) = sqrt(sqrt(sum((existStars(dd,:)-modelProjStars).^2,2)));
%             end
            %[match, cost] = munkres(dists);
            mate = maxWeightMatchingHelper(capStars, modelProjStars, modelSphereR);
            if any(mate < 0) || length(mate) < size(capStars,1)
                d = inf;
            else
                d = sum(sum((capStars - modelProjStars(mate,:)).^2,2));
            end

            %if sum(d) < bestD && (length(idx) == length(unique(idx)) || size(capStars,1) > size(existStars,1))
            %if ( cost < bestD )
            if sum(d) < bestD
                %bestD = cost;
                bestD = sum(d);
                %bestLabels = existLabels(idx);
                bestProjStars = modelProjStars;
                bestRegParams = regParams;
            end
        end
    end
end
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