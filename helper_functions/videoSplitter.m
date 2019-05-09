source_files = dir('E:\globus_data\**\*.avi'); %%replace with video files location
destination_folder = 'E:\MATLAB\cap_classifier\images'; %%replace with destination folder

for i=1:1:length(source_files)
    tokens=regexp(source_files(i).folder,filesep,'split');
    folder_name=tokens{end};
    v=VideoReader([source_files(i).folder,filesep,source_files(i).name]);
    frameNumber=1;
    folder_path = [destination_folder,filesep,folder_name];
    full_folder_path = sprintf('%s_frames', folder_path);
    % Create the folder if it doesn't exist already.
    if ~exist(full_folder_path, 'dir')
      mkdir(full_folder_path);
    end
    while hasFrame(v)
        frame = readFrame(v);
        fileName = sprintf('img%04d.jpg', frameNumber);
        outputFullFileName = fullfile(full_folder_path, fileName);
        imwrite(frame, outputFullFileName, 'jpg');
        frameNumber=frameNumber+1
    end
end


