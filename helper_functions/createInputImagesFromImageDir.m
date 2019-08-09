function createInputImagesFromImageDir(vidPath, net, imgResultFilePrefix, outputFolder, ...
    frameSkip)
%CREATEINPUTIMAGESFROMIMAGEDIR Creates a set of input images for mesh reconstruction
%(usually through VSFM)
frameCounter = 0;
imgFolder = fullfile(vidPath.folder, filesep, 'images');
fprintf("Loading images\n");
imgSet = imageSet(imgFolder);
fprintf("Loaded images, processing frames\n");
j=1;
while (j<=imgSet.Count)
    frame = read(imgSet,j);
    processVidFrame(frame, net, imgResultFilePrefix, frameCounter, outputFolder);
    frameCounter = frameCounter+frameSkip+1;
    fprintf("curFrame: %04d/%04d\n", frameCounter, imgSet.Count);
    j = j + frameSkip;
end