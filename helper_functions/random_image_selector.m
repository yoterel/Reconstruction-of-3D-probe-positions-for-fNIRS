function [fileList] = random_image_selector(infantNumbers, numberOfSamples, used_files)
if isempty(used_files)
    myUsedFiles={'DEADCODE'};
else
    myUsedFiles=used_files;
end
fileList={};
for i=1:1:numberOfSamples
    tempfile=myUsedFiles{1};
    while (any(strcmp(myUsedFiles,tempfile)))
        X = randi(length(infantNumbers));
        inputFolder = sprintf('images\\infant%d_frames\\*.jpg', infantNumbers(X));
        listing = dir(inputFolder);
        Y = randi (length(listing));
        tempfile=listing(Y).name;
    end
    fileList{end+1}=fullfile(listing(Y).folder, filesep, listing(Y).name);
    myUsedFiles{end+1}=tempfile;
end
fileList=fileList';
end