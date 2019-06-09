addpath('helper_functions', 'capnet', 'sticker_classifier', 'plyToPos', ...
    ['plyToPos', filesep, 'spm_fnirs'])

% TODO: make configurable?
% Parameters for the run
%toolPath = 'E:\MATLAB\cap_classifier\VisualSFM_windows_64bit\VisualSFM';
toolPath = '"C:\Program Files\VisualSFM_windows_64bit\VisualSFM"'; %Path of VisualSFM executable file
%sourceFiles = dir('E:\globus_data\**\*.avi'); %replace with location of raw vid files
%sourceFiles = dir('C:\Globus\emberson-consortium\VideoRecon\RESULTS\**\*.MP4'); %replace with location of raw vid files
sourceFiles = dir('C:\Globus\emberson-consortium\VideoRecon\MATLAB\model\*.MP4'); %replace with location of raw vid files
mniModelPath = "C:\Globus\emberson-consortium\VideoRecon\MATLAB\FixModelMNI.mat"; % Used by plyToPOS.m
%nirsModelPath = "C:\Globus\emberson-consortium\VideoRecon\results\infant1\NIRS_infant.mat"; % Used by plyToPos.m
nirsModelPath = "C:\Globus\emberson-consortium\VideoRecon\MATLAB\model\NIRS_model.mat"; % Used by plyToPos.m
frameSkip = 4; % How much frames to skip (process 1 frame in each frameSkip + 1 frames
useVideo = true; % Whether to use a video file directly or already extracted frame images
shimadzuFileName = "infant"; % Name of shimadzu output file in the video directory
plyToPOSgroupSize = 8; % for modelStars, 20 is used (for capStars it's fine to have outliers)
outputDirPrefix = "model";
spmPath = "C:\Users\Dean\Documents\MATLAB\spm12"; % SPM installation path

% Inner use consts
connectionsFileName = "connections.txt";
vsfmOutputFileName = "dense"; % Name of nvm file outputed by VSFM
imgResultFilePrefix = "img_result";

%%%%%%%%% Start %%%%%%%%%
data = load(fullfile('capnet', filesep, 'model.mat'));  %TODO: allow loading model from separate path?
net = data.net;

% TODO: Make this part optional?
% Take only the relevant infants
infantNumbers=1:1:1; %this array defines which infants should we process
vidSubFolders = 'E:\globus_data\infant'; %hopefully videos are in subdirectories called "infant x"
infantToSearch = [repmat(vidSubFolders, length(infantNumbers), 1), string(infantNumbers)'];
infantToSearch = strcat(infantToSearch(:,1),infantToSearch(:,2));
[Lia,Locb] = ismember(infantToSearch,{sourceFiles.folder});
index = 1;
%file_indices = Locb';
fileIndices = 1:length(sourceFiles);

for i = fileIndices
    fprintf("Processing file number %d\n", i);
    
    % TODO: use video folder name for output folder?
    % Create output directory if needed
    outputDir = sprintf('%s%sCapNet_classifier_results%s%s%d_results_stride_%d', ...
        pwd, filesep, filesep, outputDirPrefix, infantNumbers(index), frameSkip+1);
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end
    
    frameRate = createInputImages(useVideo, sourceFiles(i), net, imgResultFilePrefix, ...
        outputDir, frameSkip);    
    makeListAndConnection(outputDir, round(frameRate), frameSkip, imgResultFilePrefix, ...
        connectionsFileName);
    runVSFM(outputDir, connectionsFileName, toolPath, vsfmOutputFileName);
     
    % Video folder should also contain a stickerHSV.txt file (information on sticker locations)
    % and an infant.txt file (the Shimadzu output file)
    vidDir = sourceFiles(i).folder;
    load(strcat(vidDir, filesep, "stickerHSV.txt"), 'stickerHSV');
    %shimadzuFilePath = strcat(vidDir, filesep, shimadzuFileName, ".txt");
    shimadzuFilePath = "C:\Globus\emberson-consortium\VideoRecon\MATLAB\FixModel\CONS_subject1_sess1_20180719_134353.TXT";
    plyFilePath = strcat(outputDir, filesep, vsfmOutputFileName, ".0.ply");
    fprintf("Converting .ply file to .pos file\n");
    plyToPosOutputDir = strcat(outputDir, filesep, "plyToPosOutput");
    addpath(spmPath);
    plyToPOS(plyFilePath, stickerHSV, mniModelPath, shimadzuFilePath, plyToPosOutputDir, ...
        nirsModelPath, plyToPOSgroupSize);
    
    index = index+1;
end

function runVSFM(outputFolder, connectionsFileName, toolPath, vsfmOutputFileName)
    args = strcat(" sfm+pairs+sfm+pmvs ", outputFolder, filesep, "list.txt ", ...
        vsfmOutputFileName, ".nvm ", outputFolder, filesep, connectionsFileName);
    vsfmCmd = strcat(toolPath, args);
    fprintf("Running VisualSMF with the following command:\n%s\n", vsfmCmd);
    system(vsfmCmd);
end
