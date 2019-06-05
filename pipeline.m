addpath('helper_functions', 'capnet', 'sticker_classifier', 'plyToPos')

% TODO: make configurable?
% Parameters for the run
frameSkip = 4; %how much frames to skip
modelName = 'CapNet';
%sourceFiles = dir('E:\globus_data\**\*.avi'); %replace with location of raw vid files
sourceFiles = dir('C:\Globus\emberson-consortium\VideoRecon\MATLAB\model\MVI_0595.MP4'); %replace with location of raw vid files
%toolPath = 'E:\MATLAB\cap_classifier\VisualSFM_windows_64bit\VisualSFM';
toolPath = '"C:\Program Files\VisualSFM_windows_64bit\VisualSFM"'; %Path of VisualSFM executable file
useVideo = true;
imgResultFilePrefix = "img_result";
connectionsFileName = "connections.txt";
vsfmOutputFileName = "dense"; % Name of nvm file outputed by VSFM
mniModelPath = "C:\Globus\emberson-consortium\VideoRecon\MATLAB\FixModelMNI.mat";

%%%%%%%%%start%%%%%%%%%
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
    
    % Create output directory if needed
    outputFolder = sprintf('%s%s%s_classifier_results%sinfant%d_results_stride_%d', pwd, filesep, modelName, filesep, infantNumbers(index), frameSkip+1);
    if ~exist(outputFolder, 'dir')
      mkdir(outputFolder);
    end
    
    createInputImages(useVideo, sourceFiles(i), net, imgResultFilePrefix, imgResultFilePrefix, outputFolder);    
    makeListAndConnection(outputFolder, round(v.frameRate), frameSkip, imgResultFilePrefix, connectionsFileName); %TODO: v is undefined if not using video. What should we pass instead?
    runVSFM(outputFolder, connectionsFileName, vsfmOutputFileName); 
    %load('C:\Globus\emberson-consortium\VideoRecon\MATLAB\stickerHSV.txt', 'stickerHSV');
    %plyToPOS(sprintf("%s%s%s.0.ply", outputFolder, filesep, vsfmOutputFileName), stickerHSV, mniModelPath);
    index = index+1;
end

function runVSFM(outputFolder, connectionsFileName, toolPath, vsfmOutputFileName)
    args = strcat(" sfm+pairs+sfm+pmvs ", outputFolder, filesep, "list.txt ", vsfmOutputFileName, ".nvm ", outputFolder, filesep, connectionsFileName);
    vsfmCmd = strcat(toolPath, args);
    fprintf("Running VisualSMF with the following command:\n%s\n", vsfmCmd);
    system(vsfmCmd);
end
