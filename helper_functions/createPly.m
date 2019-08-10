function [plyFilePath] = createPly(vidPath, outputDir, vsfmOutputFileName, vsfmInputDir, ...
    toolPath, net)
%CREATEPLY Summary of this function goes here
%   Detailed explanation goes here
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end
if ~exist(vsfmInputDir, 'dir')
    mkdir(vsfmInputDir);
end

imgResultFilePrefix = "img_result";
connectionsFileName = "connections.txt";

frameRate = createInputImagesFromVideo(vidPath, net, imgResultFilePrefix, vsfmInputDir, frameSkip);    
makeListAndConnection(vsfmInputDir, round(frameRate), frameSkip, imgResultFilePrefix, ...
    connectionsFileName);
runVSFM(outputDir, vsfmInputDir, connectionsFileName, toolPath, vsfmOutputFileName);

plyFilePath = strcat(outputDir, filesep, vsfmOutputFileName, ".0.ply");
end

function runVSFM(outputFolder, inputFolder, connectionsFileName, toolPath, vsfmOutputFileName)
    args = strcat(" sfm+pairs+sfm+pmvs ", inputFolder, filesep, "list.txt ", ...
        outputFolder, filesep, vsfmOutputFileName, ".nvm ", inputFolder, filesep, ...
        connectionsFileName);
    vsfmCmd = strcat(toolPath, args);
    fprintf("Running VisualSMF with the following command:\n%s\n", vsfmCmd);
    system(vsfmCmd);
end

