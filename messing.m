addpath("nonrigidICP");
addpath("plyToPos");

load("C:\Globus\emberson-consortium\VideoRecon\MATLAB\FixModelMNI.mat", 'modelMNI');
pcshow([modelMNI.X, modelMNI.Y, modelMNI.Z]);

log("Reading ply files");
mesh1 = plyread("C:\GIT\CapNet\results\adult14_stride_5\reconstructed.ply");
mesh2 = plyread("C:\TEMP\SagiFirstCutReconPoisson.ply");
pc1 = verticesArr(mesh1);
pc2 = verticesArr(mesh2);
[scaledPc1, ~, ~] = sphereScaleAndTranslate(pc2, pc1);
hold on;
pcshow(pc1, 'r');
pcshow(pc2, 'b');
pcshow(scaledPc1, 'g');
log("Running ICP");
registered = ...
    nonrigidICPv1(verticesArr(mesh1), verticesArr(mesh2), facesArr(mesh1), facesArr(mesh2), 10 ,1);
a = 5;

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

function log(msg) %#ok<*DEFNU>
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