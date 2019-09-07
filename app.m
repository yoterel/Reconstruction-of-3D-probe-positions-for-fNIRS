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

% Last Modified by GUIDE v2.5 01-Sep-2019 15:02:14

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
addpath('helper_functions', 'capnet', 'sticker_classifier', 'plyToPos', 'TriangleRayIntersection')
end

% --- Outputs from this function are returned to the command line.
function varargout = app_OutputFcn(hObject, eventdata, handles)  %#ok<*INUSL>
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
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

% --- Executes on button press in start_btn.
function start_btn_Callback(hObject, eventdata, handles) %#ok<*DEFNU>
% hObject    handle to start_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.start_btn, 'Enable', 'off');
drawnow;
%modelMeshPath = "C:\TEMP\SagiFirstCutReconPoisson2.ply";
modelMeshPath = "C:\TEMP\SagiUpdatedAdult-Reconstructed-Edited.ply";
%plyFilePath = "C:\Globus\emberson-consortium\VideoRecon\results\adult\adult16\video1\dense.0.ply";
plyFilePath = "C:\GIT\CapNet\results\adult14_stride_5\dense.0.ply";
toolPath = '"C:\Program Files\VisualSFM_windows_64bit\VisualSFM"';
vidPath = dir('C:\Globus\emberson-consortium\VideoRecon\RESULTS\**\*.MP4');
vidPath = vidPath(1);
mniModelPath = "C:\Globus\emberson-consortium\VideoRecon\MATLAB\FixModelMNI.mat";
setProp(handles, 'mniModelPath', mniModelPath);
nirsModelPath = "C:\TEMP\NIRS_adult.mat";
setProp(handles, 'nirsModelPath', nirsModelPath);
frameSkip = 4;
stickerMinGroupSize = 5;
radiusToStickerRatio = 8;
spmPath = "C:\Users\Dean\Documents\MATLAB\spm12";
spmFNIRSPath = "C:\Users\Dean\Documents\MATLAB\spm_fnirs";
capNetModelPath = fullfile('capnet', filesep, 'model.mat');

% Name of nvm file outputed by VSFM
vsfmOutputFileName = "dense"; 

setStatusText(handles, "Loading CapNet data");
%data = load(capNetModelPath); 
%net = data.net;

% TODO: use video name/date for output folder?
% Create output directory if needed
outputDir = sprintf('%s%sresults%sadult_stride_%d', ...
    pwd, filesep, filesep, frameSkip+1);
vsfmInputDir = fullfile(outputDir, "vsfmInput");
%plyFilePath = createPly(vidPath, outputDir, vsfmOutputFileName, vsfmInputDir, toolPath, net, ...
%    frameSkip);

% Video folder should also contain a stickerHSV.txt file, which contains a
% noramlized (between 0 and 1) HSV representation of the model's sticker's
% color, and an infant.txt file (the Shimadzu output file)
vidDir = vidPath.folder;
%stickerHSVPath = strcat(vidDir, filesep, "stickerHSV.txt");
stickerHSVPath = "C:\TEMP\stickerHSV.txt";
load(stickerHSVPath, 'stickerHSV');
% Path of the shimadzu output file
shimadzuFilePath = "C:\TEMP\adult.txt";
%shimadzuFilePath = strcat(vidDir, filesep, shimadzuFileName, ".txt");

plyToPosOutputDir = strcat(outputDir, filesep, "plyToPosOutput");
addpath(spmPath, genpath(spmFNIRSPath));
setStatusText(handles, "Converting .ply file to .pos file");
%plyToPOS(plyFilePath, stickerHSV, mniModelPath, shimadzuFilePath, plyToPosOutputDir, ...
%    nirsModelPath, stickerMinGroupSize, radiusToStickerRatio);

setStatusText(handles, "Reading generated ply file");
pc = structToPointCloud(plyread(plyFilePath));
setStatusText(handles, "Reading model mesh");
modelMesh = plyread(modelMeshPath);
fM = facesArr(modelMesh);
modelPc = structToPointCloud(modelMesh, false);
showPcAndModel(handles, pc, modelPc.Location, fM);

[pc, modelPc, modelSphereR] = sphereAdjustPcToModel(handles, pc, modelPc);
[rfM, rvM] = reducepatch(fM, modelPc.Location, 8000);
showPcAndModel(handles, pc, rvM, rfM);
[pc, ~, ~, bestRmse, modelPlot] = icpAdjustPcToModel(handles, pc, modelPc, rvM, rfM);

setStatusText(handles, ...
    "Matched video ply with model, best RMSE is: %f. Calculating existing sticker positions", ...
    bestRmse);
candidates = getStickerCandidates(pc, stickerHSV);
capStars = getClosePointClusterCenters(candidates.Location, ...
    modelSphereR / radiusToStickerRatio, stickerMinGroupSize, false);
numExistingStickers = size(capStars, 1);
setProp(handles, 'numExistingStickers', numExistingStickers);
numStarsToSelect = 9 - size(capStars, 1);
pcshow(capStars, [0, 1, 0], 'MarkerSize', 50);
if numStarsToSelect < 0
    %TODO: deal with case that too many stickers were found. Change
    %parameters? (run with larger stickerMinGroupSize
elseif numStarsToSelect == 0
    onHavingEnoughStickers(handles);
else
    setStatusText(handles, "Found %d stickers, need to select %d stickers", ...
        numExistingStickers, numStarsToSelect);
    allowSelectionOnModelMesh(handles, rfM, rvM, modelPlot);
end
end

function [pcPlot, modelPlot] = showPcAndModel(handles, pc, vM, fM)
hold off;
pcPlot = pcshow(pc);
hold on;
camlight('headlight');
modelPlot = plotMesh(fM, vM);
drawnow;
end

function [tformedPc, modelPc, modelSphereR] = sphereAdjustPcToModel(handles, pc, modelPc)
%SPHEREADJUSTPCTOMODEL scaled and translates the given point cloud to match
%the model point cloud, and centers them both at the origin, based on
%approximating spheres.
setStatusText(handles, "Initial adjustments of point cloud to mesh");
pc = pcRemoveOutliers(pc); %TODO: can change stdev if necessary
[modelSphereC, modelSphereR, scale, translate] = ...
    sphereScaleAndTranslate(modelPc.Location, pc.Location);
modelPc = pctransform(modelPc, affine3d(getTransformationMatrix(1, -modelSphereC)));
tformedPc = pctransform(pc, affine3d(getTransformationMatrix(scale, ...
    translate - modelSphereC * scale)));
end

function [bestTformedPc, bestPrepRotation, bestScale, bestRmse, modelPlot] = icpAdjustPcToModel(...
    handles, pc, modelPc, vM, fM)
%ICPADJUSTPCTOMODEL 
setStatusText(handles, "Running ICP with different starting conditions");
% TODO: Combined axis rotations? Can add getRotationMatrix with 3 angles as
% helper.

% Binary search for another finer scaling based on ICP errors
lowScale = 0.7;
highScale = 1.3;
setStatusText(handles, "Performing initial ICP scalings");
[lowScalePc, lowRmse] = scaleAndICP(lowScale, pc, modelPc);
[highScalePc, highRmse] = scaleAndICP(highScale, pc, modelPc);
setStatusText(handles, "Finished initial scaling, best current rmse is: %f", ...
    min(lowRmse, highRmse));
for i = 1:5
    if lowRmse < highRmse
        highScale = (lowScale + highScale) / 2;
        [highScalePc, highRmse] = scaleAndICP(highScale, pc, modelPc);
    else
        lowScale = (lowScale + highScale) / 2;
        [lowScalePc, lowRmse] = scaleAndICP(lowScale, pc, modelPc);
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
pc = bestTformedPc;
[~, modelPlot] = showPcAndModel(handles, pc, vM, fM);

% Try ICP several times with different initial rotations of the sphere to
% try to avoid local minima problems
rotationsPerAxis = 3;
angleStep = 360 / rotationsPerAxis;
bestPrepRotation = eye(3);
for i = 1:rotationsPerAxis
    for j = 1:rotationsPerAxis
        for k = 1:rotationsPerAxis
            if i == 1 && j == 1 && k == 1
                continue;
            end
            prepRotation = rotx(i * angleStep) * roty(j * angleStep) * rotz(k * angleStep);
            rotationTform = getTransformationMatrix(1, zeros(1, 3), prepRotation);
            tmpPc = pctransform(pc, affine3d(rotationTform));
            [~, tformedPc, rmse] = pcregistericp(tmpPc, modelPc);
            if rmse < bestRmse
                bestTformedPc = tformedPc;
                bestRmse = rmse;
                bestPrepRotation = prepRotation;
                [~, modelPlot] = showPcAndModel(handles, bestTformedPc, vM, fM);
            end
            setStatusText(handles, ...
                "Finished ICP rotation iteration %d, best current rmse is: %f", ...
                rotationsPerAxis^2 * (i - 1) + rotationsPerAxis * (j - 1) + k - 1, bestRmse);
        end
    end
end
end

function [tformedPc, rmse] = scaleAndICP(scale, movingPc, fixedPc)
tform = getTransformationMatrix(scale, zeros(1, 3));
movingPc = pctransform(movingPc, affine3d(tform));
[~, tformedPc, rmse] = pcregistericp(movingPc, fixedPc);
end

function allowSelectionOnModelMesh(handles, rfM, rvM, modelPlot)
setProp(handles, 'selectedPts', []);
setProp(handles, 'isInSelectionMode', true);
ver1 = rvM(rfM(:,1),:);
ver2 = rvM(rfM(:,2),:);
ver3 = rvM(rfM(:,3),:);
set(modelPlot, 'ButtonDownFcn', @(~,~) selectPointOnMesh(handles, ver1, ver2, ver3));
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
if (~isempty(ptOnModelPlot))
    delete(ptOnModelPlot);
end
intersections = intersections(intersectionsMask, :);
if size(intersections, 1) >= 1
    ptOnModel = intersections(1, :);
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
    'FaceColor', color, 'FaceLighting', 'gouraud');
if nargin == 1
    meshPlot.VertexNormals = normals;
end
end

% --- Executes on button press in select_pt_btn.
function select_pt_btn_Callback(hObject, eventdata, handles)
% hObject    handle to select_pt_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.select_pt_btn, 'Enable', 'off');
selectedPoints = getappdata(handles.selected_pts, 'selectedPts');
numSelected = size(selectedPoints, 1);
numExistingStickers = getProp(handles, 'numExistingStickers');
numLeft = 9 - numExistingStickers - numSelected;
missingStars = getProp(handles, 'missingStars');
if numLeft > 0
    ptOnModel = getappdata(handles.selected_pts, 'ptOnModel');
    setappdata(handles.selected_pts, 'selectedPts', [selectedPoints;ptOnModel]);
    delete(getProp(handles, 'ptOnModelPlot'));
    scatter3(ptOnModel(1), ptOnModel(2), ptOnModel(3), 'filled', 'b');
    curLabel = currentPointLabel(handles);
    setProp(handles, 'missingStars', [missingStars, curLabel]);
    text(ptOnModel(1), ptOnModel(2), ptOnModel(3), curLabel, 'color', [0,0,1]);
    if numLeft > 1
        setStatusText(handles, "Found %d stickers, selected %d stickers, %d more to go", ...
            handles.numExistingStickers, numSelected + 1, numLeft - 1);
    else
        onHavingEnoughStickers(handles);
    end
else
    setProp(handles, 'isInSelectionMode', false);
    setStatusText(handles, "Cont.");
    
    load(getProp(handles, 'mniModelPath'), 'modelMNI');
    [existStars, existLabels] = findExistingStarsAndLabels(modelMNI, missingStars);
end
end

function onHavingEnoughStickers(handles)
set(handles.select_pt_btn, 'String', 'Confirm Selections');
set(handles.select_pt_btn, 'Enable', 'on');
setStatusText(handles, "No more stickers to select, confirm selections");
end

function selected_pts_CreateFcn(~, ~, ~)
end

% --- Executes on selection change in selected_pts.
function selected_pts_Callback(~, ~, handles)
% hObject    handle to selected_pts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns selected_pts contents as cell array
%        contents{get(hObject,'Value')} returns selected item from selected_pts
end

% --- Executes on button press in remove_pt_btn.
function remove_pt_btn_Callback(~, ~, handles)
% hObject    handle to remove_pt_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end

function val = getProp(handles, name)
val = getappdata(handles.selected_pts, name);
end

function setProp(handles, name, val)
setappdata(handles.selected_pts, name, val);
end

% --- Executes on selection change in point_label_popup.
function point_label_popup_Callback(~, ~, ~)
% hObject    handle to point_label_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns point_label_popup contents as cell array
%        contents{get(hObject,'Value')} returns selected item from point_label_popup
end

function label = currentPointLabel(handles)
pointLabelPopup = handles.point_label_popup;
contents = cellstr(get(pointLabelPopup, 'String'));
label = contents{get(pointLabelPopup, 'Value')};
end

% --- Executes during object creation, after setting all properties.
function point_label_popup_CreateFcn(hObject, ~, ~)
% hObject    handle to point_label_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end
