addpath("nonrigidICP");
addpath("plyToPos");
poissonReconToolPath = "C:\TEMP\PoissonRecon.exe";

log("Reading mo files");
modelMesh = plyread("C:\TEMP\SagiFirstCutReconPoisson.ply");
vM = verticesArr(modelMesh);
fM = facesArr(modelMesh);
trimesh(fM, pc2(:, 1), pc2(:, 2), pc2(:, 3));

log("Simplifying mesh");
figure;
[rf2, rv2] = reducepatch(fM, vM, 0.1);
trimesh(rf2, rv2(:, 1), rv2(:, 2), rv2(:, 3));

capMesh = plyread("C:\GIT\CapNet\results\adult14_stride_5\reconstructed.ply");
registered = nonrigidICPv1(vM, verticesArr(capMesh), fM, facesArr(capMesh), 10, 0);

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
resultsDir = "C:\GIT\CapNet\results\adult14_stride_5";
originalPcPath = fullfile(resultsDir, "dense.0.ply");
pc = plyread(originalPcPath);
pc = structToPointCloud(pc);
convertedPath = fullfile(resultsDir, "converted.ply");
pcwrite(pc, convertedPath, 'PLYFormat', 'binary');
convertedPc = pcread(convertedPath);
pcshow(convertedPc);

pc = pcread(originalPcPath);
cleaned = pcRemoveOutliers(pc);
cleanedPcPath = fullfile(resultsDir, "cleaned.ply");
pcwrite(cleaned, cleanedPcPath, 'PLYFormat', 'binary');
mesh = poissonRecon(poissonReconToolPath, cleanedPcPath, fullfile(resultsDir, "reconstructed.ply"));
end

function [pc] = structToPointCloud(pcStruct)
v = pcStruct.vertex;
colors = [v.diffuse_red, v.diffuse_green, v.diffuse_blue];
normals = [v.nx, v.ny, v.nz];
pc = pointCloud([v.x, v.y, v.z], 'Color', colors, 'Normal', normals, 'Intensity', v.psz);
end

function modelMNIMessing()
load("C:\Globus\emberson-consortium\VideoRecon\MATLAB\FixModelMNI.mat", 'modelMNI');
scatter3(modelMNI.X, modelMNI.Y, modelMNI.Z);
asPc = pointCloud([modelMNI.X, modelMNI.Y, modelMNI.Z]);
pcwrite(asPc, "C:\TEMP\modelMNI.ply", 'PLYFormat', 'binary');
mesh = poissonRecon("C:\TEMP\PoissonRecon.exe", "C:\TEMP\modelMNI.ply", "C:\TEMP\modelMNIRecon.ply");
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

function [mesh] = poissonRecon(toolPath, inputPlyPath, outputPlyPath)
system(sprintf("%s --in %s --out %s", toolPath, inputPlyPath, outputPlyPath));
mesh = plyread(outputPlyPath);
end

function log(msg)
fprintf("%s: %s\n", getCurTime(), msg);
end

function [curTime] = getCurTime()
curTime = datestr(datetime('now'));
end

function builtInICP()
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

%#ok<*DEFNU>