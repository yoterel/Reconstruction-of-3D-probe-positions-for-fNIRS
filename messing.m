addpath("nonrigidICP");
addpath("plyToPos");
poissonReconToolPath = "C:\TEMP\PoissonRecon.exe";
resultsDir = "C:\GIT\CapNet\results\adult14_stride_5";
vidPlyPath = fullfile(resultsDir, "dense.0.ply");
cleanedVidPlyPath = fullfile(resultsDir, "cleaned.ply");
reconstructedPlyPath = fullfile(resultsDir, "reconstructed3.ply");

function plyOnModelDemo(cleanedVidPlyPath, reconstructedPlyPath)
hold on;
pc = pcread(cleanedVidPlyPath);
pcshow(pc);
camlight('headlight')
mesh = plyread(reconstructedPlyPath);
[rfM, rvM] = reducepatch(facesArr(mesh), verticesArr(mesh), 5000);
plotMesh(rfM, rvM);
end

function nonrigidICPDemo(reconstructedPlyPath)
log("Reading model files");
modelMesh = plyread("C:\TEMP\SagiFirstCutReconPoisson2.ply");
vM = verticesArr(modelMesh);
fM = facesArr(modelMesh);

log("Simplifying mesh");
[rfM, rvM] = reducepatch(fM, vM, 2000);

log("Loading cap mesh");
capMesh = plyread(reconstructedPlyPath);

log("Scaling and translating cap");
vC = verticesArr(capMesh);
[vC, ~, ~] = sphereScaleAndTranslate(rvM, vC);

log("Running icp");
registered = nonrigidICPv1(rvM, vC, rfM, facesArr(capMesh), 10, 1);
save("C:\TEMP\NonRigidICPDemo1.mat", 'rvM', 'vC', 'rfM', 'capMesh', 'registered');

% load("C:\TEMP\NonRigidICPDemo1.mat", 'rvM', 'vC', 'rfM', 'capMesh', 'registered');
% hold on;
% registeredPc = pointCloud(registered, 'Color', ...
%     [capMesh.vertex.red, capMesh.vertex.green, capMesh.vertex.blue]/255);
% pcshow(registeredPc);
% plotMesh(rfM, rvM);
end

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
end

function builtInICPDemo()
vidPath = "C:\TEMP\denseNet.0.ply";
modelPath = "C:\TEMP\SagiFirstCut.ply";
vidPc = pcread(vidPath);
modelPc = pcread(modelPath);
%pcshow(vidPc);
%figure;
%hold on;
pcshow(modelPc.Location, 'r');
[tform, ~, rmse] = pcregistericp(vidPc, modelPc);
tformedPc = pctransform(vidPc, tform);
pcshow(tformedPc.Location, 'b');
end

function createCleanedPly()
pc = structToPointCloud(plyread(vidPlyPath));
cleaned = pcRemoveOutliers(pc);
pcwrite(cleaned, cleanedVidPlyPath, 'PLYFormat', 'binary');
end

function [p] = plotMesh(varargin)
%PLOTMESH
%   PLOTMESH(mesh)
%   PLOTMESH(faces, vertices)
%   PLOTMESH(faces, vertices, color)
if nargin <= 2
    color = [0.95, 0.95, 0.95];
end
if nargin == 1
    mesh = varargin{1};
    faces = facesArr(mesh);
    vertices = verticesArr(mesh);
    normals = normalsArr(mesh);
else
    faces = varargin{1};
    vertices = varargin{2};
end
p = patch('Faces', faces, 'Vertices', vertices, 'EdgeColor', 'none', ...
    'FaceColor', color, 'FaceLighting', 'gouraud');
if nargin == 1
    p.VertexNormals = normals;
end
end

function plotColoredPc(pcStruct)
v = pcStruct.vertex;
colors = [v.diffuse_red, v.diffuse_green, v.diffuse_blue];
scatter3(v.x, v.y, v.z, 1, colors);
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

function [normals] = normalsArr(plyMesh)
normals = [plyMesh.vertex.nx, plyMesh.vertex.ny, plyMesh.vertex.nz];
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
system(sprintf("%s --in %s --out %s --colors --samplesPerNode %d --normals", ...
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