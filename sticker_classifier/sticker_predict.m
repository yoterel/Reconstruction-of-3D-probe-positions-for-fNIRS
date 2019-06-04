function [segmentedFrame] = sticker_predict(frame)
    %hyper-parameters
    hueMin = 0.206;
    huMax = 0.461;
    hueRange=[hueMin, huMax];
    
    %classifier
    [hueImage, satImage, ~] = rgb2hsv(frame);
    % apply otsu thresholding for brightness
    satThreshold=graythresh(satImage);
    firstImage = satImage > satThreshold;
    secondImage = (hueImage >= hueRange(1,1) & (hueImage <= hueRange(1,2)));
    segmentedFrame = firstImage & secondImage;
    %segmentedFrame = imdilate(segmentedFrame,ones(post_inflation));
    %showImages(firstImage,secondImage,thirdImage,segmentedFrame,frame);
end

