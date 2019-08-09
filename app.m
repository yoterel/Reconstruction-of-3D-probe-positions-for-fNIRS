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

% Last Modified by GUIDE v2.5 09-Aug-2019 11:21:57

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

% Name of nvm file outputed by VSFM
vsfmOutputFileName = "dense"; 

fprintf("Loading CapNet data\n");
data = load(capNetModelPath); 
net = data.net;

% TODO: use video name/date for output folder?
% Create output directory if needed
outputDir = sprintf('%s%sresults%sadult_stride_%d', ...
    pwd, filesep, filesep, frameSkip+1);
vsfmInputDir = fullfile(outputDir, "vsfmInput");
[plyFilePath] = createPly(vidPath, outputDir, vsfmOutputFileName, vsfmInputDir, toolPath, net);

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
fprintf("Converting .ply file to .pos file\n");
plyToPOS(plyFilePath, stickerHSV, mniModelPath, shimadzuFilePath, plyToPosOutputDir, ...
    nirsModelPath, stickerMinGroupSize, radiusToStickerRatio);
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
