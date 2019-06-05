function createInputImages(useVideo, sourceFile, net, imgResultFilePrefix, outputFolder)
%CREATEINPUTIMAGES Creates a set of input images for mesh reconstruction
%(usually through VSFM)
frameCounter = 0;
if (useVideo)
    fprintf("Reading video\n");
    v = VideoReader([sourceFile.folder,filesep, sourceFile.name]);
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
    imgFolder = fullfile(sourceFile.folder,filesep, 'images');
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

