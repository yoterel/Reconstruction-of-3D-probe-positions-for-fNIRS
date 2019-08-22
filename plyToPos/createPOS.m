function createPOS(outputDir, nirsModelPath, shimadzuFilePath, modelLabels, modelOnCapCap, ...
    modelHeadIdxs, bestCapHead)
%CREATEPOS ...
%          outputDir: 
%      nirsModelPath:
%   shimadzuFilePath:
%        modelLabels:
%      modelOnCapCap:
%      modelHeadIdxs:
%        bestCapHead:
%% create files for spm_fnirs and for Homer2
fprintf("Creating files for spm_fnirs\n");
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

[Ch, Source, Detector] = importChCfgFromShimadzuTXTfile(shimadzuFilePath);
nChannels = length(Ch);
nOptodes = length(unique(Source)) + length(unique(Detector));
subX = modelOnCapCap(:,1);
subY = modelOnCapCap(:,2);
subZ = modelOnCapCap(:,3);

subX(modelHeadIdxs) = bestCapHead(:, 1);
subY(modelHeadIdxs) = bestCapHead(:, 2);
subZ(modelHeadIdxs) = bestCapHead(:, 3);

refNames = {'nz';'ar';'al';'cz';'iz'};
fid = fopen(fullfile(outputDir, "digits.txt"), 'w');
for ref = 1:length(refNames)
    ind = find(strcmpi(modelLabels, refNames{ref}));
    fprintf(fid, '%s: %f %f %f\n', refNames{ref}, subX(ind), subY(ind), subZ(ind));
end

SD.Lambda = [780;805;830];
SD.nSrcs = length(unique(Source));
SD.nDets = length(unique(Detector));
SD.SrcPos = zeros(SD.nSrcs, 3);
SD.DetPos = zeros(SD.nDets, 3);
for src = 1:SD.nSrcs
    ind = find(strcmp(modelLabels,['R',num2str(src)]));
    SD.SrcPos(src,:) = [subX(ind),subY(ind),subZ(ind)];
    fprintf(fid, 's%i: %f %f %f\n', src, subX(ind), subY(ind), subZ(ind));
end
for det = 1:SD.nDets
    ind = find(strcmp(modelLabels,['T',num2str(det)]));
    SD.DetPos(det,:) = [subX(ind),subY(ind),subZ(ind)];
    fprintf(fid, 'd%i: %f %f %f\n', src, subX(ind), subY(ind), subZ(ind));
end
fclose(fid);
SD.MeasList = [repmat([Source,Detector],3,1), ones(nChannels*3,1), ...
    [ones(nChannels,1);2*ones(nChannels,1);3*ones(nChannels,1)]];
SD.MeasListAct = ones(size(SD.MeasList, 1), 1);
SD.SpatialUnit = 'mm';
sdFilePath = fullfile(outputDir, "SD.SD");
save(sdFilePath, 'SD', '-mat')
nirsOutputFilePath = fullfile(outputDir, "nirs_model.nirs");
Shimadzu2nirsSingleFile(shimadzuFilePath, sdFilePath, nirsOutputFilePath);
txt2nirs(shimadzuFilePath);

optodePositionsFilePath = exportOptodePositions(nOptodes, modelLabels, subX, subY, subZ, outputDir);
referencePositionFilePath = exportReferencePosition(modelLabels, subX, subY, subZ, outputDir);
channelConfigOutputFilePath = exportChannelConfigOutput(Ch, Source, Detector, outputDir);

save(fullfile(outputDir, "plyToPOSoutput.mat"));

runSpmFnirs(referencePositionFilePath, optodePositionsFilePath, channelConfigOutputFilePath, ...
    nirsModelPath)
end


function [optodePositionsFilePath] = exportOptodePositions(nOptodes, subName, subX, subY, subZ, ...
    outputDir)
% EXPORTOPTODEPOSITIONS
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
outputTable = table(Optode, X, Y, Z);
optodePositionsFilePath = fullfile(outputDir, "optode_positions.csv");
writetable(outputTable, optodePositionsFilePath);
end

function [referencePositionFilePath] = exportReferencePosition(subName, subX, subY, subZ, ...
    outputDir)
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
    X(strcmpi(Reference, refNamesSpmfnirs{refInd})) = ...
        subX(strcmpi(subName, refNamesFastrak{refInd}));
    Y(strcmpi(Reference, refNamesSpmfnirs{refInd})) = ...
        subY(strcmpi(subName, refNamesFastrak{refInd}));
    Z(strcmpi(Reference, refNamesSpmfnirs{refInd})) = ...
        subZ(strcmpi(subName, refNamesFastrak{refInd}));
end
outputTable = table(Reference, X, Y, Z);
referencePositionFilePath = fullfile(outputDir, "reference_position.csv");
writetable(outputTable, referencePositionFilePath);
end

function [channelConfigOutputFilePath] = exportChannelConfigOutput(Ch, Source, Detector, outputDir)
outputTable = table(Ch, Source, Detector);
channelConfigOutputFilePath = fullfile(outputDir, "channel_config.csv");
writetable(outputTable, channelConfigOutputFilePath);
end

function [] = runSpmFnirs(referencePositionFilePath, optodePositionsFilePath, ...
    channelConfigOutputFilePath, nirsModelPath)
% RUNSPMFNIRS runs the spm_fnirs spatial tool
fprintf("Running spm_fnirs");
F{1,1}(1,:) = referencePositionFilePath;
F{1,1}(2,:) = optodePositionsFilePath;
F{1,1}(3,:) = channelConfigOutputFilePath;
F{2,1} = nirsModelPath;
spm_fnirs_spatialpreproc_ui(F);
end

