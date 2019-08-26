addpath("nonrigidICP");
addpath("plyToPos");
poissonReconToolPath = "C:\TEMP\PoissonRecon.exe";
global resultsDir;
global vidPlyPath;
global cleanedVidPlyPath;
resultsDir = "C:\GIT\CapNet\results\adult14_stride_5";
vidPlyPath = fullfile(resultsDir, "dense.0.ply");
cleanedVidPlyPath = fullfile(resultsDir, "cleaned.ply");

outputPath = fullfile(resultsDir, "reconstructed.ply");
mesh = poissonRecon(poissonReconToolPath, cleanedVidPlyPath, outputPath);
load("C:\TEMP\workspace.mat", 'modelMesh', 'capPc', 'registered');
hold on;
registeredPc = pointCloud(registered, 'Color', mesh.Color);
pcshow(registeredPc);
vM = verticesArr(modelMesh);
fM = facesArr(modelMesh);
trimesh(fM, vM(:, 1), vM(:, 2), vM(:, 3));


log("Reading model files");
modelMesh = plyread("C:\TEMP\SagiFirstCutReconPoisson2.ply");
vM = verticesArr(modelMesh);
fM = facesArr(modelMesh);
%trimesh(fM, pc2(:, 1), pc2(:, 2), pc2(:, 3));

log("Simplifying mesh");
[rf2, rv2] = reducepatch(fM, vM, 2000);
%figure;
trimesh(rf2, rv2(:, 1), rv2(:, 2), rv2(:, 3));

log("Loading cap mesh and pc");
capPc = plyread("C:\GIT\CapNet\results\adult14_stride_5\dense.0.ply");
capMesh = plyread("C:\GIT\CapNet\results\adult14_stride_5\reconstructedFromClean.ply");

log("Running icp");
registered = nonrigidICPv1(vM, verticesArr(capMesh), fM, facesArr(capMesh), 10, 1);
save("C:\TEMP\workspace.mat", 'modelMesh', 'capPc', 'capMesh', 'registered');

[scaledPc1, ~, ~] = sphereScaleAndTranslate(pc2, pc1);
hold on;
pcshow(pc1, 'r');
pcshow(pc2, 'b');
pcshow(scaledPc1, 'g');
log("Running ICP");
registered = nonrigidICPv1(... 
    verticesArr(capMesh), verticesArr(modelMesh), facesArr(capMesh), facesArr(modelMesh), 10 ,1);
a = 5;

function convertStructToPcDemo()
pc = plyread(vidPlyPath);
pc = structToPointCloud(pc);
convertedPath = fullfile(resultsDir, "converted.ply");
pcwrite(pc, convertedPath, 'PLYFormat', 'binary');
end

function modelMNIDemo()
load("C:\Globus\emberson-consortium\VideoRecon\MATLAB\FixModelMNI.mat", 'modelMNI');
scatter3(modelMNI.X, modelMNI.Y, modelMNI.Z);
asPc = pointCloud([modelMNI.X, modelMNI.Y, modelMNI.Z]);
pcwrite(asPc, "C:\TEMP\modelMNI.ply", 'PLYFormat', 'binary');
mesh = poissonRecon("C:\TEMP\PoissonRecon.exe", "C:\TEMP\modelMNI.ply", "C:\TEMP\modelMNIRecon.ply");
end

function builtInICPDemo()
vidPath = "C:\TEMP\denseNet.0.ply";
modelPath = "C:\TEMP\SagiFirstCut.ply";
vidPc = pcread(vidPath);
modelPc = pcread(modelPath);
pcshow(vidPc);
figure;
hold on;
pcshow([modelPc.Location(:,1), modelPc.Location(:,2), modelPc.Location(:,3)], 'r');
tform = pcregistericp(vidPc, modelPc);
tformedPc = pctransform(vidPc, tform);
pcshow([tformedPc.Location(:,1), tformedPc.Location(:,2), tformedPc.Location(:,3)], 'b');
end

function createCleanedPly()
pc = structToPointCloud(plyread(vidPlyPath));
cleaned = pcRemoveOutliers(pc);
pcwrite(cleaned, cleanedVidPlyPath, 'PLYFormat', 'binary');
end

function plotColoredPc(pcStruct)
v = pcStruct.vertex;
colors = [v.diffuse_red, v.diffuse_green, v.diffuse_blue];
scatter3(v.x, v.y, v.z, 1, colors);
end

function [pc] = structToPointCloud(pcStruct)
v = pcStruct.vertex;
colors = [v.diffuse_red, v.diffuse_green, v.diffuse_blue];
normals = [v.nx, v.ny, v.nz];
pc = pointCloud([v.x, v.y, v.z], 'Color', colors/255, 'Normal', normals);
end

function pcRemoveOutliersDemo()
pc = verticesArr(plyread("C:\GIT\CapNet\results\adult14_stride_5\dense.0.ply"));
pcshow(pc);
figure;
pcshow(pcRemoveOutliers(pc, 3));
figure;
pcshow(pcRemoveOutliers(pc, 2));
figure;
pcshow(pcRemoveOutliers(pc, 1.5));
end

function [faces] = facesArr(plyMesh)
faces = cell2mat(plyMesh.face.vertex_indices) + 1;
end

function [vertices] = verticesArr(plyMesh)
vertices = [plyMesh.vertex.x, plyMesh.vertex.y, plyMesh.vertex.z];
end

function [mesh] = poissonRecon(toolPath, inputPlyPath, outputPlyPath, convertColors, samplesPerNode)
if nargin >= 4 && convertColors
    originalPcStruct = plyread(inputPlyPath);
    originalPc = structToPointCloud(originalPcStruct);
    inputPlyPathDir = dir(inputPlyPath);
    tmpPcFilePath = fullfile(inputPlyPathDir.folder, "tmp.ply");
    pcwrite(originalPc, tmpPcFilePath, 'PLYFormat', 'binary');
    inputPlyPath = tmpPcFilePath;
end
if nargin < 5
    samplesPerNode = 5;
end
system(sprintf("%s --in %s --out %s --colors --samplesPerNode %d", ...
    toolPath, inputPlyPath, outputPlyPath, samplesPerNode));
mesh = plyread(outputPlyPath);
end

function log(msg)
fprintf("%s: %s\n", getCurTime(), msg);
end

function [curTime] = getCurTime()
curTime = datestr(datetime('now'));
end

%#ok<*DEFNU>