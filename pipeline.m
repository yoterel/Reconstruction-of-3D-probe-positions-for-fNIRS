addpath('helper_functions', 'capnet', 'sticker_classifier')

frame_skip = 4; %how much frames to skip
model_name = 'CapNet';
start_frame_number = 0;
%source_files = dir('E:\globus_data\**\*.avi'); %replace with location of raw vid files
source_files = dir('C:\Globus\emberson-consortium\VideoRecon\MATLAB\model\MVI_0595.MP4'); %replace with location of raw vid files
%tool_path = 'E:\MATLAB\cap_classifier\VisualSFM_windows_64bit\VisualSFM';
tool_path = 'C:\Program Files\VisualSFM_windows_64bit\VisualSFM'; %Path of VisualSFM executable file
%if not edit code below to extract only infant which user required above ("infant_numbers")
use_video=true; %TODO: make parameter?
imgResultFilePrefix = "img_result";
connectionsFileName = "connections.txt";

%%%%%%%%%start%%%%%%%%%
data = load(fullfile('capnet', filesep, 'model.mat'));  %TODO: allow loading model from separate path?
net = data.net;

% TODO: Make this part optional?
% Take only the relevant infants
infant_numbers=1:1:1; %this array defines which infants should we process
vid_sub_folders = 'E:\globus_data\infant'; %hopefully videos are in subdirectories called "infant x"
infant_to_search = [repmat(vid_sub_folders, length(infant_numbers), 1), string(infant_numbers)'];
infant_to_search = strcat(infant_to_search(:,1),infant_to_search(:,2));
[Lia,Locb] = ismember(infant_to_search,{source_files.folder});
index=1;
%file_indices = Locb';
file_indices = 1:length(source_files);

for i=file_indices
    fprintf("Processing file number %d\n", i);
    
    % Create output directory if needed
    folder = pwd;   % Make it a subfolder of the folder where this m-file lives.
    outputFolder = sprintf('%s%s%s_classifier_results%sinfant%d_results_stride_%d', pwd, filesep, model_name, filesep, infant_numbers(index), frame_skip+1);
    if ~exist(outputFolder, 'dir')
      mkdir(outputFolder);
    end
    frameCounter = 0;
        
    if (use_video)
        fprintf("Reading video\n");
        v = VideoReader([source_files(i).folder,filesep, source_files(i).name]);
        fprintf("Finished reading video, processing frames\n");
        while hasFrame(v)
            frame = readFrame(v);
            processFrame(frame, net, imgResultFilePrefix, frameCounter, outputFolder);
            frameCounter = frameCounter+frame_skip+1;
            fprintf("curTime: %d curFrame: %d\n", v.CurrentTime, frameCounter);
            temp_counter=0;
            while (temp_counter<frame_skip)
                try
                    frame=readFrame(v);
                    temp_counter=temp_counter+1;
                catch ME
                    break;
                end
            end   
        end
    else
        imgFolder = fullfile(source_files(i).folder,filesep, 'images');
        fprintf("Loading images\n");
        imgSet = imageSet(imgFolder);
        fprintf("Loaded images, processing frames\n");
        j=1;
        while (j<=imgSet.Count)
            frame = read(imgSet,j);
            processFrame(frame, net, imgResultFilePrefix, frameCounter, outputFolder);
            frameCounter = frameCounter+frame_skip+1;
            fprintf("curFrame: %d\n", frameCounter);
            j = j + frame_skip;
        end
    end
    makeListAndConnection(outputFolder, round(v.frameRate), frame_skip, imgResultFilePrefix, connectionsFileName);
    command_line = [' sfm+pairs+sfm+pmvs ', outputFolder, filesep, 'list.txt',' dense.nvm ', outputFolder, filesep, connectionsFileName];
    system([tool_path, command_line]);
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

%chomp video frames and process them,saving results to output folder
function writeImages(maskedRgbImage, finalImage, frameCounter, outputFolder, imgResultFilePrefix)
    fileName1 = sprintf('img_mask%04d.jpg', frameCounter);
    fileName2 = sprintf('%s%04d.jpg', imgResultFilePrefix, frameCounter);
    outputFullFileName = fullfile(outputFolder, fileName1);
    imwrite(finalImage, outputFullFileName, 'jpg');
    outputFullFileName = fullfile(outputFolder, fileName2);
    imwrite(maskedRgbImage, outputFullFileName, 'jpg'); 
end
