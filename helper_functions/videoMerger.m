outputFolder = sprintf('%s', pwd); %replace with folder with images
imageNames = dir([outputFolder,'/img_result*.jpg']);
imageNames = {imageNames.name}';
outputVideo = VideoWriter('infant9_masked_10.avi');
outputVideo.FrameRate = 240;
open(outputVideo)
for ii = 1:length(imageNames)
   ii
   img = imread([outputFolder,'/',imageNames{ii}]);
   writeVideo(outputVideo,img)
end
close(outputVideo)