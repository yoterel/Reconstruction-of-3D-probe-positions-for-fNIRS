%TODO: add paths of subfolders directly in code
frame_skip=4; %how much frames to skip
infant_numbers=1:1:1; %this array defines which infants should we process
model_name='CapNet';
start_frame_number=0;
%source_files = dir('E:\globus_data\**\*.avi'); %replace with location of raw vid files
source_files = dir('C:\Globus\emberson-consortium\VideoRecon\MATLAB\model\MVI_0595.MP4'); %replace with location of raw vid files
vid_sub_folders= 'E:\globus_data\infant' %hopefully videos are in subdirectories called "infant x"
%tool_path='E:\MATLAB\cap_classifier\VisualSFM_windows_64bit\VisualSFM';
tool_path='C:\Program Files\VisualSFM_windows_64bit\VisualSFM'; %Path of VisualSFM executable file
%if not edit code below to extract only infant which user required above ("infant_numbers")
use_video=true; %TODO: make parameter?
%%%%%%%%%start%%%%%%%%%
data = load(fullfile('capnet', filesep, 'model.mat'));  %TODO: allow loading model from separate path?
net = data.net;

% TODO: Make this optional in case we want to filter only specific infants
infant_to_search=[repmat('E:\globus_data\infant',length(infant_numbers),1),string(infant_numbers)']; %take only infants mentioned
infant_to_search=strcat(infant_to_search(:,1),infant_to_search(:,2));
[Lia,Locb]=ismember(infant_to_search,{source_files.folder});
index=1;
%for i=1:1:1
for i=Locb'
    frameCounter=0;
    %create output directory if needed
    folder = pwd;   % Make it a subfolder of the folder where this m-file lives.
    outputFolder = sprintf('%s/%s_classifier_results/infant%d_results_stride_%d', pwd, model_name, infant_numbers(index), frame_skip+1);
    if ~exist(outputFolder, 'dir')
      mkdir(outputFolder);
    end
    if (use_video)
        v=VideoReader([source_files(i).folder,filesep, source_files(i).name]);
        while hasFrame(v)
            frame=readFrame(v);
            capImage=capnet_predict(frame,net);
            stickerImage=sticker_predict(frame);
            erodedImage=imerode(stickerImage,ones(10));
            finalImage = imdilate(erodedImage,ones(10)) | capImage;
            maskedRgbImage = bsxfun(@times, frame, cast(finalImage, 'like', frame));
            writeImages(maskedRgbImage,finalImage,frameCounter,outputFolder);
            frameCounter=frameCounter+frame_skip+1;
            disp(['curTime: ', num2str(v.CurrentTime),' curFrame: ', num2str(frameCounter)]);
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
        imgFolder=fullfile(source_files(i).folder,filesep,'images');
        imgSet = imageSet(imgFolder);
        j=1;
        while (j<=imgSet.Count)
            frame=read(imgSet,j);
            capImage=CapNet_classifier(frame,net);
            stickerImage=sticker_classifier(frame);
            erodedImage=imerode(stickerImage,ones(10));
            finalImage = imdilate(erodedImage,ones(10)) | capImage;
            maskedRgbImage = bsxfun(@times, frame, cast(finalImage, 'like', frame));
            writeImages(maskedRgbImage,finalImage,frameCounter,outputFolder);
            frameCounter=frameCounter+frame_skip+1;
            disp(['curTime: ', num2str(v.CurrentTime),' curFrame: ', num2str(frameCounter)]);
            j=j+frame_skip;
        end
    end
    makeListAndConnection(outputFolder,round(v.frameRate),frame_skip); %this function is under helper_functions
    command_line=[' sfm+pairs+sfm+pmvs ', outputFolder,filesep,'list.txt',' dense.nvm ',outputFolder,filesep, 'connections.txt'];
    system([tool_path, command_line]);
    index=index+1;
end
%chomp video frames and process them,saving results to output folder
function writeImages(maskedRgbImage,finalImage,frameCounter, outputFolder)
    fileName1 = sprintf('img_mask%04d.jpg', frameCounter);
    fileName2 = sprintf('img_result%04d.jpg', frameCounter);
    outputFullFileName = fullfile(outputFolder, fileName1);
    imwrite(finalImage, outputFullFileName, 'jpg');
    outputFullFileName = fullfile(outputFolder, fileName2);
    imwrite(maskedRgbImage, outputFullFileName, 'jpg'); 
end
