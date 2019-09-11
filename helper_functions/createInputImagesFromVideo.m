function [frameRate] = createInputImagesFromVideo(vidPath, net, imgResultFilePrefix, ...
    outputFolder, frameSkip, logFunc)
%CREATEINPUTIMAGESFROMVIDEO Creates a set of input images for mesh reconstruction
%(usually through VSFM)
frameCounter = 0;
logFunc("Reading video");
v = VideoReader([vidPath.folder, filesep, vidPath.name]);
frameRate = v.frameRate; % TODO: what shold we return as frameRate if not using video???
numberOfFrames = round(frameRate * v.Duration);
logFunc("Finished reading video, processing frames");
while hasFrame(v)
    frame = readFrame(v);
    processVidFrame(frame, net, imgResultFilePrefix, frameCounter, outputFolder);
    frameCounter = frameCounter + frameSkip + 1;
    logFunc("Current time: %0.3f seconds, current frame: %04d/%04d", ...
		v.CurrentTime, frameCounter, numberOfFrames);
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