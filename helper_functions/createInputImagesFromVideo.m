function [frameRate] = createInputImagesFromVideo(vidPath, net, imgResultFilePrefix, ...
    outputFolder, frameSkip)
%CREATEINPUTIMAGESFROMVIDEO Creates a set of input images for mesh reconstruction
%(usually through VSFM)
frameCounter = 0;
fprintf("Reading video\n");
v = VideoReader([vidPath.folder, filesep, vidPath.name]);
frameRate = v.frameRate; % TODO: what shold we return as frameRate if not using video???
numberOfFrames = round(frameRate * v.Duration);
fprintf("Finished reading video, processing frames\n");
while hasFrame(v)
    frame = readFrame(v);
    processVidFrame(frame, net, imgResultFilePrefix, frameCounter, outputFolder);
    frameCounter = frameCounter + frameSkip + 1;
    fprintf("curTime: %d seconds, curFrame: %04d/%04d\n", v.CurrentTime, frameCounter, ...
        numberOfFrames);
     tempCounter=0;
     while (tempCounter < frameSkip)
        try
            readFrame(v);
            tempCounter = tempCounter+1;
        catch
            break;
        end
    end   
end