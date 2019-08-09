function processVidFrame(frame, net, imgResultFilePrefix, frameCounter, outputFolder)
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