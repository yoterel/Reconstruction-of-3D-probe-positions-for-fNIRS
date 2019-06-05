function [stickerHSV] = getStickerHSV(img,colorName)
imshow(img);
if (nargin == 1)
    colorName = 'sticker';
end
title(['Select a point on ' colorName ' ... Press SPACE if non found']);
HideCursor;
[x,y] = ginput(1);
ShowCursor;
RGB = uint8(zeros(1,3));
RGB(:) = img(floor(y),floor(x),:);
stickerHSV = rgb2hsv(double(RGB)/255);