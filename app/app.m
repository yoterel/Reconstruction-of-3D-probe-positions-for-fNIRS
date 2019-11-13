function varargout = app(varargin)
% APP MATLAB code for app.fig
%      APP, by itself, creates a new APP or raises the existing
%      singleton*.
%
%      H = APP returns the handle to a new APP or the handle to
%      the existing singleton*.
%
%      APP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in APP.M with the given input arguments.
%
%      APP('Property','Value',...) creates a new APP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before app_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to app_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help app

% Last Modified by GUIDE v2.5 25-Oct-2019 18:06:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @app_OpeningFcn, ...
                   'gui_OutputFcn',  @app_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
end

% --- Executes just before app is made visible.
function app_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to app (see VARARGIN)

% Choose default command line output for app
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes app wait for user response (see UIRESUME)
% uiwait(handles.figure1);
addpath('helper_functions', 'capnet', 'sticker_classifier', 'plyToPos', ...
    'app/TriangleRayIntersection');
setProp(handles, 'varargin', varargin);
end

% --- Outputs from this function are returned to the command line.
function varargout = app_OutputFcn(hObject, eventdata, handles)  %#ok<*INUSL>
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
varargin = getProp(handles, 'varargin');
modelMeshPath = varargin{3};
toolPath = sprintf("""%s""", varargin{4});
vidPath = dir(varargin{2});
vidPath = vidPath(1);
mniModelPath = varargin{7};
nirsModelPath = varargin{5};
setProp(handles, 'nirsModelPath', nirsModelPath);
frameSkip = varargin{11};
stickerMinGroupSize = varargin{12};
radiusToStickerRatio = varargin{13};
spmPath = varargin{6};
spmFNIRSPath = varargin{8};
capNetModelPath = fullfile('capnet', filesep, 'model.mat');
outputDir = varargin{14};
[plyExists, plyFile] = checkOutputDir(outputDir);
plyToPosOutputDir = strcat(outputDir, filesep, "plyToPosOutput");
setProp(handles, 'outputDir', plyToPosOutputDir);

setStatusText(handles, "Loading CapNet data");
data = load(capNetModelPath); 
net = data.net;

% Name of nvm file outputed by VSFM
if plyExists
    answer = questdlg('A ply file exists in output folder, Do you want to use it and skip ply creation step?', ...
        'Use existing ply?', 'Yes', 'No', 'Yes');
    % Handle response
    switch answer
        case 'Yes'
            plyFilePath = plyFile;
        case 'No'
            vsfmOutputFileName = "dense"; 
            vsfmInputDir = fullfile(outputDir, "vsfmInput");
            plyFilePath = createPly(vidPath, outputDir, vsfmOutputFileName, vsfmInputDir, toolPath, net, ...
                frameSkip, @(msg, varargin) setStatusText(handles, msg, varargin{:}));
    end    
end

% TODO: decide if in video foler or not
% Video folder should also contain a stickerHSV.txt file, which contains a
% noramlized (between 0 and 1) HSV representation of the model's sticker's
% color, and an infant.txt file (the Shimadzu output file)
%vidDir = vidPath.folder;
%stickerHSVPath = strcat(vidDir, filesep, "stickerHSV.txt");
stickerHSVPath = varargin{9};
load(stickerHSVPath, 'stickerHSV');
% Path of the shimadzu output file
shimadzuFilePath = varargin{10};
setProp(handles, 'shimadzuFilePath', shimadzuFilePath);

addpath(spmPath, genpath(spmFNIRSPath));

setStatusText(handles, "Reading generated ply file");
pc = structToPointCloud(plyread(plyFilePath));
pcshow(pc);
pc = cleanGeneratedPointCloud(handles, pc);

setStatusText(handles, "Reading model mesh");
modelMesh = plyread(modelMeshPath);

setStatusText(handles, "Simplifying model mesh");
[rfM, rvM] = reducepatch(facesArr(modelMesh), verticesArr(modelMesh), 8000);
showPcAndModel(handles, pc, rvM, rfM);

[modelMNIPoints, modelMNILabels] = loadModelMNI(mniModelPath);
[pc, rvM, modelSphereR, modelMNIPoints] = sphereAdjustPcToModel(handles, pc, rvM, modelMNIPoints);
setProp(handles, 'modelSphereR', modelSphereR);
setProp(handles, 'modelMNIPoints', modelMNIPoints);
setProp(handles, 'modelMNILabels', modelMNILabels);
showPcAndModel(handles, pc, rvM, rfM);
[pc, ~, ~, bestRmse] = icpAdjustPcToModel(handles, pc, rvM, rfM);

setProp(handles, 'vM', rvM);
setProp(handles, 'fM', rfM);
setProp(handles, 'stickerHSV', stickerHSV);
setProp(handles, 'radiusToStickerRatio', radiusToStickerRatio);
setProp(handles, 'stickerMinGroupSize', stickerMinGroupSize);
postIcpMatch(handles, bestRmse, stickerHSV, pc, modelSphereR, radiusToStickerRatio, ...
    stickerMinGroupSize, rfM, rvM);
set(handles.tryagainbtn, 'Enable', 'on');
end

function [plyExists, plyFile] = checkOutputDir(outputDir)
plyExists = false;
fileList = dir(fullfile(outputDir, filesep, '*.ply'));
listSize = size(fileList);
if listSize(1) == 1
    plyExists = true;
    plyFile = fullfile(outputDir, filesep, fileList.name);
end
end

function postIcpMatch(handles, bestRmse, stickerHSV, pc, modelSphereR, radiusToStickerRatio, ...
    stickerMinGroupSize, rfM, rvM)
setStatusText(handles, ...
    "Matched video ply with model, best RMSE is: %f. Calculating existing sticker positions", ...
    bestRmse);
[capStars, numExistingStickers, numStarsToSelect] = calculateCapStars(handles, stickerHSV, pc, ...
    modelSphereR, radiusToStickerRatio, stickerMinGroupSize);
if numStarsToSelect < 0
    %TODO: deal with case that too many stickers were found. Change
    %parameters? (run with larger stickerMinGroupSize
elseif numStarsToSelect == 0
    onHavingEnoughStickers(handles);
else
    setStatusText(handles, "Found %d stickers, need to select %d stickers", ...
        numExistingStickers, numStarsToSelect);
    allowSelectionOnModelMesh(handles, rfM, rvM, pc);
end
% This must be executed after allowSelectionOnModelMesh, which first
% clears the current plot
pcshow(capStars, [0, 1, 0], 'MarkerSize', 100);
end

function pc = cleanGeneratedPointCloud(handles, pc)
%CLEANGENERATEDPOINTCLOUD Removes black points which are often noise, and
%removes points that are too far away from the center of mass
setStatusText(handles, "Initial cleaning of the generated point cloud");
pc = pcRemoveOutliers(pc);
colors = pc.Color;
r = colors(:, 1);
g = colors(:, 2);
b = colors(:, 3);
mask = (r > 50) | (g > 70) | (b > 50);
pc = filterPcPoints(pc, mask);
pcshow(pc);
end

function [pcPlot, modelPlot] = showPcAndModel(handles, pc, vM, fM)
hold off;
pcPlot = pcshow(pc);
hold on;
camlight('headlight');
modelPlot = plotMesh(fM, vM);
drawnow;
end

function [modelMNIPoints, modelMNILabels] = loadModelMNI(mniModelPath)
load(mniModelPath, 'modelMNI');
modelMNIPoints = [modelMNI.X, modelMNI.Y, modelMNI.Z]; 
modelMNILabels = modelMNI.labels;
end

function [tformedPc, vM, modelSphereR, modelMNI] = sphereAdjustPcToModel(handles, pc, vM, modelMNI)
%SPHEREADJUSTPCTOMODEL scaled and translates the given point cloud to match
%the model point cloud, and centers them both at the origin, based on
%approximating spheres.
setStatusText(handles, "Using approximating spheres to adjust point cloud to model");
[modelSphereC, modelSphereR, scale, translate] = sphereScaleAndTranslate(vM, pc.Location);
vM = vM - modelSphereC;
modelMNI = modelMNI - modelSphereC;
tformMatrix = getTransformationMatrix(scale, translate - modelSphereC * scale);
tformedPc = pctransform(pc, affine3d(tformMatrix));
end

function [bestTformedPc, bestPrepRotation, bestScale, bestRmse] = icpAdjustPcToModel(handles, ...
    pc, vM, fM)
%ICPADJUSTPCTOMODEL 
setStatusText(handles, "Running ICP with different starting conditions");

modelPc = pointCloud(vM);
[pc, ~, ~] = combinedRotationsICP(handles, pc, modelPc, inf, vM, fM, 2);

% Binary search for another finer scaling based on ICP errors
[bestTformedPc, bestRmse, bestScale] = icpFindBestScale(handles, 0.7, 1.3, pc, modelPc, 5);
pc = bestTformedPc;
showPcAndModel(handles, pc, vM, fM);

% Try ICP several times with different initial rotations to try to avoid
% local minima problems
[pc, ~, bestPrepRotation] = singleAxisRotationsICP(...
    handles, pc, modelPc, bestRmse, vM, fM, 12, 12, 12);
% [pc, ~, bestPrepRotation] = combinedRotationsICP(...
%     handles, pc, modelPc, bestRmse, vM, fM, 3);

[bestTformedPc, bestRmse, bestScale2] = icpFindBestScale(handles, 0.9, 1.1, pc, modelPc, 3);
bestScale = bestScale * bestScale2;
end

function [bestTformedPc, bestRmse, bestScale] = icpFindBestScale(...
    handles, lowScale, highScale, pc, modelPc, iterations)
%ICPFINDBESTSCALE Performs a binary search for finding the best scale,
%based on ICP errors
setStatusText(handles, "Performing initial ICP scalings");
[lowScalePc, lowRmse] = tformAndICP(pc, modelPc, lowScale);
[highScalePc, highRmse] = tformAndICP(pc, modelPc, highScale);
setStatusText(handles, "Finished initial scaling, best current rmse is: %f", ...
    min(lowRmse, highRmse));
for i = 1:iterations
    if lowRmse < highRmse
        highScale = (lowScale + highScale) / 2;
        [highScalePc, highRmse] = tformAndICP(pc, modelPc, highScale);
    else
        lowScale = (lowScale + highScale) / 2;
        [lowScalePc, lowRmse] = tformAndICP(pc, modelPc, lowScale);
    end
    setStatusText(handles, "Finished ICP scaling iteration %d, best current rmse is: %f", ...
        i, min(lowRmse, highRmse));
end
if lowRmse < highRmse
    bestTformedPc = lowScalePc;
    bestRmse = lowRmse;
    bestScale = lowScale;
else
    bestTformedPc = highScalePc;
    bestRmse = highRmse;
    bestScale = highScale;
end
end

function [tformedPc, rmse, icpTform] = tformAndICP(movingPc, fixedPc, scale, rotation)
if nargin < 4
    rotation = eye(3);
end
tform = getTransformationMatrix(scale, zeros(1, 3), rotation);
movingPc = pctransform(movingPc, affine3d(tform));
[icpTform, tformedPc, rmse] = pcregistericp(movingPc, fixedPc);
end

function [bestTformedPc, bestRmse, bestPrepRotation] = combinedRotationsICP(...
    handles, pc, modelPc, bestRmse, vM, fM, rotationsPerAxis, changeThreshold)
%COMBINEDROTATIONICP Runs ICP several times with different initial
%rotations of the given point cloud in several axes to try to avoid local
%minima problems. The number of rotations is rotationsPerAxis^3 - 1: all
%combinations of rotations per axis for 3 axes, except the trivial rotation
%which is assumed to already be tested.
angleStep = 360 / rotationsPerAxis;
bestPrepRotation = eye(3);
bestTformedPc = pc;
if nargin < 8
    changeThreshold = 0;
end
for i = 1:rotationsPerAxis
    for j = 1:rotationsPerAxis
        for k = 1:rotationsPerAxis
            if i == 1 && j == 1 && k == 1
                continue;
            end
            prepRotation = rotx(i * angleStep) * roty(j * angleStep) * rotz(k * angleStep);
            [tformedPc, rmse, icpTform] = tformAndICP(pc, modelPc, 1, prepRotation);
            if rmse < bestRmse && getChangeValue(icpTform, prepRotation) >= changeThreshold
                bestTformedPc = tformedPc;
                bestRmse = rmse;
                bestPrepRotation = prepRotation;
                showPcAndModel(handles, bestTformedPc, vM, fM);
            end
            setStatusText(handles, ...
                "Finished ICP rotation iteration %d, best current rmse is: %f", ...
                rotationsPerAxis^2 * (i - 1) + rotationsPerAxis * (j - 1) + k - 1, bestRmse);
        end
    end
end
end

function [bestTformedPc, bestRmse, bestPrepRotation] = singleAxisRotationsICP(...
    handles, pc, modelPc, bestRmse, vM, fM, rotationsX, rotationsY, rotationsZ, changeThreshold)
%COMBINEDROTATIONICP Runs ICP several times with different initial
%rotations of the given point cloud (each rotation in a single axis) to try
%to avoid local minima problems. The number of rotations is rotationsX +
%rotationsY + rotationsZ - 3 (trivial rotations aren't tested).
bestPrepRotation = eye(3);
bestTformedPc = pc;
if nargin < 10
    changeThreshold = 0;
end

angleStep = 360 / rotationsX;
for i = 1:(rotationsX - 1)
    prepRotation = rotx(i * angleStep);
    [tformedPc, rmse, icpTform] = tformAndICP(pc, modelPc, 1, prepRotation);
    if rmse < bestRmse && getChangeValue(icpTform, prepRotation) >= changeThreshold
        bestTformedPc = tformedPc;
        bestRmse = rmse;
        bestPrepRotation = prepRotation;
        showPcAndModel(handles, bestTformedPc, vM, fM);
    end
    setStatusText(handles, "Finished ICP rotation iteration %d, best current rmse is: %f", ...
        i, bestRmse);
end

angleStep = 360 / rotationsY;
for i = 1:(rotationsY - 1)
    prepRotation = roty(i * angleStep);
    [tformedPc, rmse, icpTform] = tformAndICP(pc, modelPc, 1, prepRotation);
    if rmse < bestRmse && getChangeValue(icpTform, prepRotation) >= changeThreshold
        bestTformedPc = tformedPc;
        bestRmse = rmse;
        bestPrepRotation = prepRotation;
        showPcAndModel(handles, bestTformedPc, vM, fM);
    end
    setStatusText(handles, "Finished ICP rotation iteration %d, best current rmse is: %f", ...
        i + rotationsX - 1, bestRmse);
end

angleStep = 360 / rotationsZ;
for i = 1:(rotationsZ - 1)
    prepRotation = rotz(i * angleStep);
    [tformedPc, rmse, icpTform] = tformAndICP(pc, modelPc, 1, prepRotation);
    if rmse < bestRmse && getChangeValue(icpTform, prepRotation) >= changeThreshold
        bestTformedPc = tformedPc;
        bestRmse = rmse;
        bestPrepRotation = prepRotation;
        showPcAndModel(handles, bestTformedPc, vM, fM);
    end
    setStatusText(handles, "Finished ICP rotation iteration %d, best current rmse is: %f", ...
        i + rotationsX + rotationsY - 2, bestRmse);
end
end

function allowSelectionOnModelMesh(handles, rfM, rvM, pc)
[~, modelPlot] = showPcAndModel(handles, pc, rvM, rfM);
setProp(handles, 'selectedPts', []);
setProp(handles, 'isInSelectionMode', true);
ver1 = rvM(rfM(:,1),:);
ver2 = rvM(rfM(:,2),:);
ver3 = rvM(rfM(:,3),:);
set(modelPlot, 'ButtonDownFcn', @(~,~) selectPointOnMesh(handles, ver1, ver2, ver3));
setProp(handles, 'pc', pc);
end

function selectPointOnMesh(handles, ver1, ver2, ver3)
isInSelectionMode = getProp(handles, 'isInSelectionMode');
if ~isInSelectionMode
    return
end
hold on;
curPointMat = get(gca, 'CurrentPoint');
orig = curPointMat(1,:);
direction = curPointMat(2,:) - orig;
[intersectionsMask, ~, ~, ~, intersections] = TriangleRayIntersection(...
    orig, direction, ver1, ver2, ver3);
ptOnModelPlot = getProp(handles, 'ptOnModelPlot');
if ~isempty(ptOnModelPlot)
    delete(ptOnModelPlot);
end
intersections = intersections(intersectionsMask, :);
if size(intersections, 1) >= 1
    [~, closestIntersectionIndex] = min(vecnorm((intersections - orig)'));
    ptOnModel = intersections(closestIntersectionIndex, :);
    setProp(handles, 'ptOnModelPlot', ...
        scatter3(ptOnModel(1), ptOnModel(2), ptOnModel(3), 'filled', 'r'));
    setProp(handles, 'ptOnModel', ptOnModel);
    set(handles.select_pt_btn, 'Enable', 'on');
end
end

function faces = facesArr(plyMesh)
faces = cell2mat(plyMesh.face.vertex_indices) + 1;
end

function vertices = verticesArr(plyMesh)
vertices = [plyMesh.vertex.x, plyMesh.vertex.y, plyMesh.vertex.z];
end

function normals = normalsArr(plyMesh)
normals = [plyMesh.vertex.nx, plyMesh.vertex.ny, plyMesh.vertex.nz];
end

function meshPlot = plotMesh(varargin)
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
meshPlot = patch('Faces', faces, 'Vertices', vertices, 'EdgeColor', 'none', ...
    'FaceColor', color, 'FaceLighting', 'gouraud', 'FaceAlpha', 0.5);
if nargin == 1
    meshPlot.VertexNormals = normals;
end
end

function [capStars, numExistingStickers, numStarsToSelect] = calculateCapStars(handles, ...
    stickerHSV, pc, modelSphereR, radiusToStickerRatio, stickerMinGroupSize)
candidates = getStickerCandidates(pc, stickerHSV);
capStars = getClosePointClusterCenters(candidates.Location, ...
    modelSphereR / radiusToStickerRatio, stickerMinGroupSize, false);
setProp(handles, 'capStars', capStars);
numExistingStickers = size(capStars, 1);
setProp(handles, 'numExistingStickers', numExistingStickers);
numStarsToSelect = 9 - numExistingStickers;
end

% --- Executes on button press in select_pt_btn.
function select_pt_btn_Callback(hObject, eventdata, handles)
% hObject    handle to select_pt_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.select_pt_btn, 'Enable', 'off');
selectedPoints = getProp(handles, 'selectedPts');
numSelected = size(selectedPoints, 1);
numExistingStickers = getProp(handles, 'numExistingStickers');
numLeft = 9 - numExistingStickers - numSelected;
if numLeft > 0
    ptOnModel = getProp(handles, 'ptOnModel');
    setProp(handles, 'selectedPts', [selectedPoints;ptOnModel]);
    delete(getProp(handles, 'ptOnModelPlot'));
    hold on;
    scatter3(ptOnModel(1), ptOnModel(2), ptOnModel(3), 'filled', 'b');
    if numLeft > 1
        setStatusText(handles, "Found %d stickers, selected %d stickers, %d more to go", ...
            numExistingStickers, numSelected + 1, numLeft - 1);
    else
        onHavingEnoughStickers(handles);
    end
else
    onConfirmedAllStarSelections(handles, selectedPoints, numSelected);
end
end

function onConfirmedAllStarSelections(handles, selectedPoints, numSelected)
setProp(handles, 'isInSelectionMode', false);
setStatusText(handles, "Labeling stickers");

modelMNIPoints = getProp(handles, 'modelMNIPoints');
modelMNILabels = getProp(handles, 'modelMNILabels');
numModelMNIPoints = size(modelMNIPoints, 1);
headAndCapIdxs = (numModelMNIPoints-8):numModelMNIPoints; 
modelStars = modelMNIPoints(headAndCapIdxs, :); % Coordinates of stars on model
modelStarLabels = modelMNILabels(headAndCapIdxs); % Ordered labels of stars on model

modelSphereR = getProp(handles, 'modelSphereR');
capStars = getProp(handles, 'capStars');
capStars = [capStars;selectedPoints];
mate = minDistanceMatchPoints(capStars, modelStars, modelSphereR);
if any(mate < 0) || length(mate) < size(capStars, 1)
    % The case where the matching isn't complete (should occur
    % only if some points are further apart than the sphere's
    % radius).
    error("Couldn't match labels to the cap stars");
end

% If successful regulation parameters were found, then bestMate should be
% the same length as capStars, meaning we labeled all existing stars
capLabels = modelStarLabels(mate);
text(capStars(:, 1), capStars(:, 2), capStars(:, 3), capLabels, 'color', [0,1,0]);
setStatusText(handles, ...
    "Successfuly labeled stickers, performing grid search of axis-aligned scaling");

% Discard manually selected stickers for the upcoming optimizations
numExisting = 9 - numSelected;
missingStars = capLabels((numExisting + 1):9);
capLabels = capLabels(1:numExisting);
existStarsBitVec = ~ismember(modelStarLabels, missingStars);
existLabels = modelStarLabels(existStarsBitVec);

% reorder capStars and capLabels according to modelLabels
tempCapStars = capStars;
for i = 1:length(existLabels)
    capStars(i,:) = tempCapStars(strcmp(capLabels, existLabels{i}), :);
end
capLabels = existLabels;

% TODO: use consts for point labels?
headLabelNames = {'Nz', 'Cz', 'AR', 'AL'};
capHeadIdxs = ismember(capLabels, headLabelNames);
existPointsBitVec = ~ismember(modelMNILabels, missingStars);
modelHeadIdxs = ismember(modelMNILabels, headLabelNames) & existPointsBitVec;
capLabelNames = {'Front', 'Cz', 'Right', 'Left', 'Pz', 'Iz'};
capCapIdxs = ismember(capLabels, capLabelNames);
modelCapIdxs = ismember(modelMNILabels, capLabelNames) & existPointsBitVec;
capHead = capStars(capHeadIdxs,:);
modelHead = modelMNIPoints(modelHeadIdxs,:);
[bestReg, capHead, bestScale] = findHeadTransformation(capHead, modelHead);
setStatusText(handles, 'Applying calculated transformations');
capStars = applyRegParams(capStars.*bestScale, bestReg);

% rotate scaled model to match the best to the CAP POINTS ONLY.
modelCap = modelMNIPoints(modelCapIdxs,:);
modelStarsToErase = true(length(modelCap(:,1)),1);
capCapLabels = capLabels(capCapIdxs);
modelCapLabels = modelMNILabels(modelCapIdxs);
for i = 1:size(modelCapLabels,1)
    if any(ismember(capCapLabels, modelCapLabels{i}))
        modelStarsToErase(i) = false;
    end
end
modelCap(modelStarsToErase,:) = [];
capCap = capStars(capCapIdxs,:);
[modelReg, ~] = absor(modelCap',capCap');
modelOnCap = applyRegParams(modelMNIPoints, modelReg);

[subX, subY, subZ] = getCapOnHeadPositions(modelOnCap, capHead, modelHeadIdxs);
shimadzuFilePath = getProp(handles, 'shimadzuFilePath');
nirsModelPath = getProp(handles, 'nirsModelPath');
outputDir = getProp(handles, 'outputDir');
setStatusText(handles, "Final preparations + running spm to create POS file");
figure;
createPOS(outputDir, nirsModelPath, shimadzuFilePath, modelMNILabels, subX, subY, subZ);
end

function onHavingEnoughStickers(handles)
set(handles.select_pt_btn, 'String', 'Confirm Selections');
set(handles.select_pt_btn, 'Enable', 'on');
setStatusText(handles, "No more stickers to select, confirm selections");
end

function val = getProp(handles, name)
val = getappdata(handles.select_pt_btn, name);
end

function setProp(handles, name, val)
setappdata(handles.select_pt_btn, name, val);
end

function compare_pc_and_model(handles, plyFilePath, modelMeshPath)
setStatusText(handles, "Reading generated ply file");
pc = structToPointCloud(plyread(plyFilePath));
setStatusText(handles, "Reading model mesh");
modelMesh = plyread(modelMeshPath);
setStatusText(handles, "Plotting pc and mesh");
vM = verticesArr(modelMesh);
fM = facesArr(modelMesh);
pcshow(pc);
allowSelectionOnModelMesh(handles, fM, vM);
drawnow;
end

function pc_on_model_demo(handles)
resultsDir = "C:\GIT\CapNet\results\adult14_stride_5";
cleanedVidPlyPath = fullfile(resultsDir, "cleaned.ply");
reconstructedPlyPath = fullfile(resultsDir, "reconstructed3.ply");
hold on;
pc = pcread(cleanedVidPlyPath);
pcshow(pc);
camlight('headlight');
mesh = plyread(reconstructedPlyPath);
[rfM, rvM] = reducepatch(facesArr(mesh), verticesArr(mesh), 5000);
vert1 = rvM(rfM(:,1),:);
vert2 = rvM(rfM(:,2),:);
vert3 = rvM(rfM(:,3),:);

function selectPointOnMesh(~, ~)
    hold on;
    clickedPt = get(gca,'CurrentPoint');
    msg = sprintf("[%.3f,%.3f,%.3f]\n[%.3f,%.3f,%.3f]", ...
        clickedPt(1,1), clickedPt(1,2), clickedPt(1,3), ...
        clickedPt(2,1), clickedPt(2,2), clickedPt(2,3)); 
    setStatusText(handles, msg);
    
    if (isfield(handles, 'front_pt'))
        delete(handles.front_pt);
    end
    handles.front_pt = scatter3(clickedPt(1,1), clickedPt(1,2), clickedPt(1,3), 'filled', 'r');
    
    if (isfield(handles, 'back_pt'))
        delete(handles.back_pt);
    end
    handles.back_pt = scatter3(clickedPt(2,1), clickedPt(2,2), clickedPt(2,3), 'filled', 'b');
    
    orig = clickedPt(1,:);
    dir = clickedPt(2,:) - orig;
    
    if isfield(handles, 'line')
        delete(handles.line);
    end
    handles.line = line('XData' ,orig(1)+[0 dir(1)], 'YData', orig(2)+[0 dir(2)], 'ZData', ...
        orig(3)+[0 dir(3)], 'Color', 'r', 'LineWidth', 3);
    
    [intersectionsMask, ~, ~, ~, intersections] = TriangleRayIntersection(...
        orig, dir, vert1, vert2, vert3);
    if (isfield(handles, 'intersection'))
        delete(handles.intersection);
    end
    intersections = intersections(intersectionsMask, :);
    if size(intersections, 1) >= 1
        intersection = intersections(1, :);
        handles.intersection = scatter3(...
            intersection(1), intersection(2), intersection(3), 'filled', 'g');
    end
end

meshPlot = plotMesh(rfM, rvM);
set(meshPlot, 'ButtonDownFcn', @selectPointOnMesh);
drawnow;
end

% --- Executes on button press in tryagainbtn.
function tryagainbtn_Callback(hObject, eventdata, handles)
% hObject    handle to tryagainbtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(hObject, 'Enable', 'off');
pc = getProp(handles, 'pc');
vM = getProp(handles, 'vM');
fM = getProp(handles, 'fM');
stickerHSV = getProp(handles, 'stickerHSV');
modelSphereR = getProp(handles, 'modelSphereR');
radiusToStickerRatio = getProp(handles, 'radiusToStickerRatio');
stickerMinGroupSize = getProp(handles, 'stickerMinGroupSize');

changeThreshold = 0.4;
rotationsPerSingleAxis = 8;
modelPc = pointCloud(vM);
[pc1, rmse1, ~] = singleAxisRotationsICP(handles, pc, modelPc, inf, vM, fM, ...
    rotationsPerSingleAxis, rotationsPerSingleAxis, rotationsPerSingleAxis, changeThreshold);
rotationsPerAxis = 4;
[pc2, rmse2, ~] = combinedRotationsICP(handles, pc, modelPc, inf, vM, fM, ...
    rotationsPerAxis, changeThreshold);
if rmse1 < rmse2
    pc = pc1;
    bestRmse = rmse1;
else
    pc = pc2;
    bestRmse = rmse2;
end
if bestRmse == inf
    error("Couldn't find a sufficiently different matching");
end
postIcpMatch(handles, bestRmse, stickerHSV, pc, modelSphereR, radiusToStickerRatio, ...
    stickerMinGroupSize, fM, vM)
end

function change = getChangeValue(tform, prepRotation)
tformMatrix = tform.T;
rotation = tformMatrix(1:3,1:3) * prepRotation;
change = 1 - max(rotation(1,1), rotation(2,2));
end