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

% Last Modified by GUIDE v2.5 10-Aug-2019 12:38:42

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
addpath('helper_functions', 'capnet', 'sticker_classifier', 'plyToPos')
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

function pc_on_model_demo()
resultsDir = "C:\GIT\CapNet\results\adult14_stride_5";
cleanedVidPlyPath = fullfile(resultsDir, "cleaned.ply");
reconstructedPlyPath = fullfile(resultsDir, "reconstructed3.ply");
hold on;
pc = pcread(cleanedVidPlyPath);
pcshow(pc);
camlight('headlight')
mesh = plyread(reconstructedPlyPath);
[rfM, rvM] = reducepatch(facesArr(mesh), verticesArr(mesh), 5000);
plotMesh(rfM, rvM);
drawnow;
end

% --- Executes on button press in start_btn.
function start_btn_Callback(hObject, eventdata, handles) %#ok<*DEFNU>
set(handles.start_btn, 'Enable', 'off');
drawnow;

% hObject    handle to start_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
toolPath = '"C:\Program Files\VisualSFM_windows_64bit\VisualSFM"';
vidPath = dir('C:\Globus\emberson-consortium\VideoRecon\RESULTS\**\*.MP4');
vidPath = vidPath(1);
mniModelPath = "C:\Globus\emberson-consortium\VideoRecon\MATLAB\FixModelMNI.mat";
nirsModelPath = "C:\TEMP\NIRS_adult.mat";
frameSkip = 4;
stickerMinGroupSize = 5;
radiusToStickerRatio = 8;
spmPath = "C:\Users\Dean\Documents\MATLAB\spm12";
spmFNIRSPath = "C:\Users\Dean\Documents\MATLAB\spm_fnirs";
capNetModelPath = fullfile('capnet', filesep, 'model.mat');
modelMeshPath = "C:\TEMP\SagiFirstCutReconPoisson2.ply";

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
plyFilePath = "C:\TEMP\denseNet.0.ply";

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
vM = verticesArr(modelMesh);
fM = facesArr(modelMesh);
modelPc = structToPointCloud(modelMesh, false);

setStatusText(handles, "Initial adjustments of point cloud to mesh");
pc = pcRemoveOutliers(pc); %TODO: can change stdev if necessary
[~, ~, scale, translate] = sphereScaleAndTranslate(vM, pc.Location);
pc = pctransform(pc, affine3d(getTransformationMatrix(scale, translate)));

setStatusText(handles, "Running ICP with different starting conditions");
% TODO: add y/z axis rotations? can use roty, rotz. Can add
% getRotationMatrix with 3 angles as helper. 

% Try ICP several times with different initial rotations of the sphere to
% try to avoid local minima problems
numRotations = 4;
[~, bestTformedPc, bestRmse] = pcregistericp(pc, modelPc);
for i = 1:(numRotations - 1)
    rotationTform = getTransformationMatrix(1, zeros(1, 3), rotx(i * 360 / numRotations));
    tmpPc = pctransform(pc, affine3d(rotationTform));
    [~, tformedPc, rmse] = pcregistericp(tmpPc, modelPc);
    if rmse < bestRmse
        bestTformedPc = tformedPc;
        bestRmse = rmse;
    end
end
pc = bestTformedPc;

setStatusText(handles, "Matched video ply with model, calculating existing sticker positions");
candidates = getStickerCandidates(pc, stickerHSV);

function selectPointOnMesh(~, ~)
    hold on;
    clickedPt = get(gca,'CurrentPoint');
    msg = sprintf("[%.3f,%.3f,%.3f]\n[%.3f,%.3f,%.3f]", ...
        clickedPt(1,1), clickedPt(1,2), clickedPt(1,3), ...
        clickedPt(2,1), clickedPt(2,2), clickedPt(2,3)); 
    setStatusText(handles, msg);
    if (isfield(handles, 'clicked_pt'))
        delete(handles.clicked_pt);
    end
    handles.clicked_pt = scatter3(clickedPt(1,1), clickedPt(1,2), clickedPt(1,3), 'filled', 'r');
end

hold on;
pcshow(pc);
camlight('headlight');
[rfM, rvM] = reducepatch(fM, vM, 5000);
meshPlot = plotMesh(rfM, rvM);
set(meshPlot, 'ButtonDownFcn', @selectPointOnMesh);

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

% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over start_btn.
function start_btn_ButtonDownFcn(hObject, ~, handles)
% hObject    handle to start_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end
