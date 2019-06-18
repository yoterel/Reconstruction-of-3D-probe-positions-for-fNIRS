function [] = plyToPOS(fullPlyFileName, stickerHSV, mniModelPath, shimadzuFilePath, outputDir, ...
    nirsModelPath, stickerMinGroupSize, radiusToStickerRatio)
%capStars = [capStars; [-7.888 3.4265 -2.053]*modelSphereR/capSphereR+modelSphereC-capSphereC];
%manualPoints = [-4.561 0.562 3.81]; %subject 2
%manualPoints = [-1.246 1.953 -1.643; -0.893 1.65 -1.731]; %2783a
manualPoints = [];

%% Get hue of stickers and locate relevant points in the ply
mesh = plyread(fullPlyFileName);
fprintf("Finished reading ply file\n");
vertices = [mesh.vertex.x mesh.vertex.y mesh.vertex.z];
verColors = [mesh.vertex.diffuse_red mesh.vertex.diffuse_green mesh.vertex.diffuse_blue];
verColorsHSV = rgb2hsv(double(verColors)/255);
trgShade = stickerHSV(1);
idxs = find(abs(verColorsHSV(:,1)-trgShade) < 0.15 & ((verColorsHSV(:,2) > 0.2 & verColorsHSV(:,3) > 0.2) | (verColorsHSV(:,2) > 0.3 & verColorsHSV(:,3) > 0.1)));
candidates = vertices(idxs,:);
plot3(vertices(idxs,1),vertices(idxs,2),vertices(idxs,3),'.');
[capSphereC, capSphereR] = sphereFit(candidates);
fprintf("Performed sphere fit 1\n");
%load(['model',filesep,'modelStars.mat']); %load modelStars, modelLabels, modelSphereC,modelSphereR
load(mniModelPath, 'modelMNI');
%load(mniModelPath, 'infantModelMNI');
%modelMNI = infantModelMNI;
fprintf("Loaded MNI model\n");

headAndCapIdxs = length(modelMNI.X)-8:length(modelMNI.X);
modelStars = [modelMNI.X(headAndCapIdxs),modelMNI.Y(headAndCapIdxs),modelMNI.Z(headAndCapIdxs)]; 
modelLabels = modelMNI.labels(headAndCapIdxs);
[modelSphereC,modelSphereR] = sphereFit([modelMNI.X,modelMNI.Y,modelMNI.Z]);
fprintf("Performed sphere fit 2\n")
%modelStars = modelStars*capSphereR/modelSphereR-modelSphereC+capSphereC; % scale and translate model to fit cap
candidates = candidates * modelSphereR / capSphereR+modelSphereC - capSphereC;

%% Divide sticker points into clouds
% bbox = [max(vertices); min(vertices)];
% radius = sqrt(sum((bbox(1,:)-bbox(2,:)).^2));
radius = modelSphereR;
thr = radius / radiusToStickerRatio;
distMatrix = zeros(length(candidates));
adjMatrix = zeros(length(candidates));
for v = 1:length(candidates)
    distMatrix(:,v) = sqrt(sum([candidates(:,1)-candidates(v,1) candidates(:,2)-candidates(v,2) candidates(:,3)-candidates(v,3)].^2,2));
end
adjMatrix(distMatrix < thr) = 1;
[labels, ~] = graphConnectedComponents(adjMatrix);

%% Convert sticker point clouds into positions, calcualte size of each sticker (in points), remove small groups
figure; axis equal; hold on
counter = zeros(max(labels),1);
mids = zeros(max(labels),3);
for i = 1:max(labels)
    scatter3(candidates(labels == i,1),candidates(labels == i,2),candidates(labels == i,3));
    counter(i) = size(candidates(labels == i,1),1);
    mids(i,:) = sum(candidates(labels == i,:),1)./counter(i);
end
% [c,ia,ic] = unique(labels);
% d = [true; diff(ic) ~= 0; true];  % TRUE if values change
% n = diff(find(d));               % Number of repetitions
% rts = find(n > groupSize);
% rts = n(rts);
%%
%plot3(mids(counter > groupSize,1),mids(counter > groupSize,2),mids(counter > groupSize,3),'p','markersize',25,'MarkerFaceColor','k');
% X = [mesh.vertex.x,mesh.vertex.y,mesh.vertex.z];
% X = X - mean(X);
% plot3(X(:,1),X(:,2),X(:,3),'.');
% Throw away stickers that are too small
capStars = mids(counter > stickerMinGroupSize,:); 

%% Add the manually added points
if ~isempty(manualPoints)
    capStars = [capStars; manualPoints*modelSphereR/capSphereR+modelSphereC-capSphereC];
end

%%
%capStars = capStars - mean(capStars);% implicit expansion
plot3(capStars(:,1),capStars(:,2),capStars(:,3),'p','markersize',25,'MarkerFaceColor','k');
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
% capFront = capStars(4,:);
% capLeft = capStars(3,:);
% capRight = capStars(5,:);
% testCapTriplet = [capFront;capRight;capLeft];
capTripletOrder = nchoosek(1:size(capStars,1),3);
% capTripletOrder = [4,3,5];
% modelTriplet = [modelStars(strcmpi(modelLabels,'front'),:);modelStars(strcmpi(modelLabels,'right'),:);modelStars(strcmpi(modelLabels,'left'),:)];
% missingStars = {'Nz';'Iz';'AR';'AL'};
missingStars = {}; % TODO: this should be marked by user. We have to validate we have 9 points overall
existStars = modelStars(~ismember(modelLabels,missingStars),:);
%modelTriplet = modelTriplet - mean(existStars);
%existStars = existStars - mean(existStars);
existLabels = modelLabels(~ismember(modelLabels,missingStars));
modelTripletOrder = nchoosek(1:size(existStars,1),3);
w = 0.7;
bestD = inf;
outlierThr = capSphereR;
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

%% Use bestRegParams and bestProjStars to adjust the model
fprintf("Applying the calculated regulation parameters\n");
mate = maxWeightMatchingHelper(capStars, bestProjStars, modelSphereR);
capStars(mate < 1,:) = [];
mate(mate < 1) = [];
capLabels = existLabels(mate);

%capLabels = existLabels((idx(I(IA))));
figure;
hold on;axis equal;
plot3(capStars(:,1),capStars(:,2),capStars(:,3),'ob')
text(capStars(:,1),capStars(:,2),capStars(:,3),capLabels,'color',[0,0,1])
plot3(bestProjStars(:,1),bestProjStars(:,2),bestProjStars(:,3),'pr')
text(bestProjStars(:,1),bestProjStars(:,2),bestProjStars(:,3),existLabels,'color',[1,0,0])

% rotate model to MNI. Use this rotation for the cap
capStars = [capStars ones(length(capStars),1)]*(bestRegParams.M^-1)';
capStars = capStars(:,1:3);
modelPoints = [modelMNI.X,modelMNI.Y,modelMNI.Z]; 
modelLabels = [modelMNI.labels];

% reorder capStars and capLabels according to modelLabels
tempCapStars = capStars;
for ii = 1:length(existLabels)
    capStars(ii,:) = tempCapStars(strcmp(capLabels,existLabels{ii}),:);
end
capLabels = existLabels;

%% perform grid search of axis-aligned scaling, and realign at each step.
% USING ONLY THE HEAD POINTS
% TODO: use consts for point labels?
fprintf("Performing grid search of axis-aligned scaling\n");
capHeadIdxs = ismember(capLabels,{'Nz','Cz','AR','AL'});
modelHeadIdxs = ismember(modelLabels,{'Nz','Cz','AR','AL'}) & ~ismember(modelLabels, missingStars);
capCapIdxs = ismember(capLabels,{'Front','Cz','Right','Left','Pz','Iz'});
modelCapIdxs = ismember(modelLabels,{'Front','Cz','Right','Left','Pz','Iz'})& ...
    ~ismember(modelLabels,missingStars);

modelHead = modelPoints(modelHeadIdxs,:);
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
            [idx,d] = knnsearch(modelHead,testStars); 
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
[modelReg, movedModel] = absor(modelCap',capCap');
modelOnCapCap = [modelPoints ones(size(modelPoints,1),1)]*modelReg.M';
modelOnCapCap(:,4) = [];

%% Graph results
figure; hold on; axis equal; scatter3(modelHead(:,1),modelHead(:,2),modelHead(:,3));
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