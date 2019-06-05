addpath('helper_functions', 'capnet', 'sticker_classifier')

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
    folder = pwd;   % Make it a subfolder of the folder where this m-file lives.
    outputFolder = sprintf('%s%s%s_classifier_results%sinfant%d_results_stride_%d', pwd, filesep, modelName, filesep, infantNumbers(index), frameSkip+1);
    if ~exist(outputFolder, 'dir')
      mkdir(outputFolder);
    end
    
    frameCounter = 0;
    if (useVideo)
        fprintf("Reading video\n");
        v = VideoReader([sourceFiles(i).folder,filesep, sourceFiles(i).name]);
        fprintf("Finished reading video, processing frames\n");
        while hasFrame(v)
            frame = readFrame(v);
            processFrame(frame, net, imgResultFilePrefix, frameCounter, outputFolder);
            frameCounter = frameCounter+frameSkip+1;
            fprintf("curTime: %d curFrame: %d\n", v.CurrentTime, frameCounter);
            tempCounter=0;
            while (tempCounter < frameSkip)
                try
                    frame = readFrame(v);
                    tempCounter = tempCounter+1;
                catch ME
                    break;
                end
            end   
        end
    else
        imgFolder = fullfile(sourceFiles(i).folder,filesep, 'images');
        fprintf("Loading images\n");
        imgSet = imageSet(imgFolder);
        fprintf("Loaded images, processing frames\n");
        j=1;
        while (j<=imgSet.Count)
            frame = read(imgSet,j);
            processFrame(frame, net, imgResultFilePrefix, frameCounter, outputFolder);
            frameCounter = frameCounter+frameSkip+1;
            fprintf("curFrame: %d\n", frameCounter);
            j = j + frameSkip;
        end
    end
    
    makeListAndConnection(outputFolder, round(v.frameRate), frameSkip, imgResultFilePrefix, connectionsFileName);
    runVSFM(outputFolder, connectionsFileName);
    index = index+1;
end 

function processFrame(frame, net, imgResultFilePrefix, frameCounter, outputFolder)
    capImage = capnet_predict(frame, net);
    stickerImage = sticker_predict(frame);
    erodedImage = imerode(stickerImage, ones(10));
    finalImage = imdilate(erodedImage, ones(10)) | capImage;
    maskedRgbImage = bsxfun(@times, frame, cast(finalImage, 'like', frame));
    writeImages(maskedRgbImage, finalImage, frameCounter, outputFolder, imgResultFilePrefix);    
end

% Saves 2 image results from a frame (finalImage + maskedRgbImage to output folder
function writeImages(maskedRgbImage, finalImage, frameCounter, outputFolder, imgResultFilePrefix)
    fileName1 = sprintf('img_mask%04d.jpg', frameCounter);
    fileName2 = sprintf('%s%04d.jpg', imgResultFilePrefix, frameCounter);
    outputFullFileName = fullfile(outputFolder, fileName1);
    imwrite(finalImage, outputFullFileName, 'jpg');
    outputFullFileName = fullfile(outputFolder, fileName2);
    imwrite(maskedRgbImage, outputFullFileName, 'jpg'); 
end

function runVSFM(outputFolder, connectionsFileName, toolPath)
    args = strcat(" sfm+pairs+sfm+pmvs ", outputFolder, filesep, 'list.txt', " dense.nvm ", outputFolder, filesep, connectionsFileName);
    vsfmCmd = strcat(toolPath, args);
    fprintf("Running VisualSMF with the following command:\n%s\n", vsfmCmd);
    system(vsfmCmd);
end
