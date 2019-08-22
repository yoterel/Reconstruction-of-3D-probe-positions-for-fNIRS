addpath('helper_functions', 'capnet', 'sticker_classifier', 'plyToPos')

% TODO: make configurable? Possibly use .ini file
% Parameters for the run

% Path of VisualSFM executable file
%toolPath = 'E:\MATLAB\cap_classifier\VisualSFM_windows_64bit\VisualSFM';
toolPath = '"C:\Program Files\VisualSFM_windows_64bit\VisualSFM"'; 

% Location of raw video file
%vidPath = dir('E:\globus_data\**\*.avi')(1);
%vidPath = dir('C:\Globus\emberson-consortium\VideoRecon\results\**\*.MP4');
vidPath = dir('C:\Globus\emberson-consortium\VideoRecon\results\adult\adult14\video1\*.MP4');
vidPath = vidPath(1);

% Contains info about the model of the cap itself, used by plyToPOS
%mniModelPath = "C:\Globus\emberson-consortium\VideoRecon\MATLAB\infantModelMNI.mat";
mniModelPath = "C:\Globus\emberson-consortium\VideoRecon\MATLAB\FixModelMNI.mat";

% Used by plyToPOS
nirsModelPath = "C:\TEMP\NIRS_adult.mat";
%nirsModelPath = "C:\Globus\emberson-consortium\VideoRecon\MATLAB\model\NIRS_model.mat";

% The number of frames to skip when reading video (process 1 frame in each frameSkip + 1 frames
frameSkip = 4;

% Minimum number of points in a group for it to be considered a separate
% sticker. For modelStars, 20 is used (for capStars it's fine to have outliers)
stickerMinGroupSize = 5;

% Relative distance for which 2 points are considered the same sticker
radiusToStickerRatio = 8;

% spm installation path
spmPath = "C:\Users\Dean\Documents\MATLAB\spm12";

% spm_fnirs installation path
spmFNIRSPath = "C:\Users\Dean\Documents\MATLAB\spm_fnirs";

% Model for the CapNet video classifier
capNetModelPath = fullfile('capnet', filesep, 'model.mat');

% Name of nvm file outputed by VSFM
vsfmOutputFileName = "dense";

%%%%%%%%% Start %%%%%%%%%
fprintf("Loading CapNet data\n");
data = load(capNetModelPath); 
net = data.net;

% TODO: use video name/date for output folder?
outputDir = sprintf('%s%sresults%sadult14_stride_%d', ...
    pwd, filesep, filesep, frameSkip+1);
vsfmInputDir = fullfile(outputDir, "vsfmInput");
%plyFilePath = createPly(vidPath, outputDir, vsfmOutputFileName, vsfmInputDir, toolPath, net, frameSkip);
plyFilePath = "C:\TEMP\denseNet.0.ply";
%plyFilePath ="C:\GIT\CapNet\results\adult14_stride_5\dense.0.ply";

% Video folder should also contain a stickerHSV.txt file, which contains a
% noramlized (between 0 and 1) HSV representation of the model's sticker's
% color, and an infant.txt file (the Shimadzu output file)
vidDir = vidPath.folder;
%stickerHSVPath = strcat(vidDir, filesep, "stickerHSV.txt");
stickerHSVPath = "C:\TEMP\stickerHSV.txt";
load(stickerHSVPath, 'stickerHSV');
% Path of the shimadzu output file
shimadzuFilePath = "C:\TEMP\adult.txt";
%shimadzuFilePath = strcat(vidDir, filesep, shimadzuFileName, ".txt");

plyToPosOutputDir = strcat(outputDir, filesep, "plyToPosOutput");
addpath(spmPath, genpath(spmFNIRSPath));
fprintf("Converting .ply file to .pos file\n");
plyToPOS(plyFilePath, stickerHSV, mniModelPath, shimadzuFilePath, plyToPosOutputDir, ...
    nirsModelPath, stickerMinGroupSize, radiusToStickerRatio);