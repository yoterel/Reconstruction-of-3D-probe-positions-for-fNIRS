function [plyFilePath] = createPly(vidPath, outputDir, vsfmOutputFileName, vsfmInputDir, ...
    toolPath, net, frameSkip, logFunc)
%CREATEPLY Summary of this function goes here
%   Detailed explanation goes here
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end
if ~exist(vsfmInputDir, 'dir')
    mkdir(vsfmInputDir);
end
if nargin < 8
    logFunc = @(msg, varargin) fprintf(msg + "\n", varargin{:});
end

imgResultFilePrefix = "img_result";
connectionsFileName = "connections.txt";

frameRate = createInputImagesFromVideo(vidPath, net, imgResultFilePrefix, vsfmInputDir, ...
    frameSkip, logFunc);
makeListAndConnection(vsfmInputDir, round(frameRate), frameSkip, imgResultFilePrefix, ...
    connectionsFileName, logFunc);
runVSFM(outputDir, vsfmInputDir, connectionsFileName, toolPath, vsfmOutputFileName, logFunc);

plyFilePath = strcat(outputDir, filesep, vsfmOutputFileName, ".0.ply");
end

function runVSFM(outputFolder, inputFolder, connectionsFileName, toolPath, vsfmOutputFileName, ...
    logFunc)
    args = strcat(" sfm+pairs+sfm+pmvs ", inputFolder, filesep, "list.txt ", ...
        outputFolder, filesep, vsfmOutputFileName, ".nvm ", inputFolder, filesep, ...
        connectionsFileName);
    vsfmCmd = strcat(toolPath, args);
    logFunc("Running VisualSMF with the following command:\n%s\n", vsfmCmd);
    system(vsfmCmd);
end

