function varargout = startWindow(varargin)
% STARTWINDOW MATLAB code for startWindow.fig
%      STARTWINDOW, by itself, creates a new STARTWINDOW or raises the existing
%      singleton*.
%
%      H = STARTWINDOW returns the handle to a new STARTWINDOW or the handle to
%      the existing singleton*.
%
%      STARTWINDOW('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in STARTWINDOW.M with the given input arguments.
%
%      STARTWINDOW('Property','Value',...) creates a new STARTWINDOW or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before startWindow_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to startWindow_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help startWindow

% Last Modified by GUIDE v2.5 11-Sep-2019 12:22:21

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @startWindow_OpeningFcn, ...
                   'gui_OutputFcn',  @startWindow_OutputFcn, ...
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
%#ok<*DEFNU>

% --- Executes just before startWindow is made visible.
function startWindow_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to startWindow (see VARARGIN)

% Choose default command line output for startWindow
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes startWindow wait for user response (see UIRESUME)
% uiwait(handles.startWindow);

% --- Outputs from this function are returned to the command line.
function varargout = startWindow_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in submitBtn.
function submitBtn_Callback(hObject, ~, handles)
% hObject    handle to submitBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(hObject, 'Enable', 'off');
videoPath = get(handles.vidPathInput, 'String');
modelMeshPath = get(handles.modelMeshPathInput, 'String');
vsfmPath = get(handles.vsfmPathInput, 'String');
nirsModelPath = get(handles.nirsModelPathInput, 'String');
spmPath = get(handles.spmPathInput, 'String');
mniModelPath = get(handles.mniModelPathInput, 'String');
spmFNIRSPath = get(handles.spmFNIRSPathInput, 'String');
stickerHSVPath = get(handles.stickerHSVPathInput, 'String');
shimadzuFilePath = get(handles.shimadzuFilePathInput, 'String');
frameSkip = str2double(get(handles.frameSkipInput, 'String'));
stickerMinGroupSize = str2double(get(handles.stickerMinGroupSizeInput, 'String'));
radiusToStickerRatio = str2double(get(handles.radiusToStickerRatioInput, 'String'));
% First parameter is nonsense to avoid stupid MATLAB warning in case it is
% a string
app(0, videoPath, modelMeshPath, vsfmPath, nirsModelPath, spmPath, mniModelPath, spmFNIRSPath, ...
    stickerHSVPath, shimadzuFilePath, frameSkip, stickerMinGroupSize, radiusToStickerRatio);

function vidPathInput_Callback(~, ~, ~)
% hObject    handle to vidPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function vidPathInput_CreateFcn(hObject, ~, ~)
% hObject    handle to vidPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function mniModelPathInput_Callback(~, ~, ~)
% hObject    handle to mniModelPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function mniModelPathInput_CreateFcn(hObject, ~, ~)
% hObject    handle to mniModelPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function nirsModelPathInput_Callback(~, ~, ~)
% hObject    handle to nirsModelPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function nirsModelPathInput_CreateFcn(hObject, ~, ~)
% hObject    handle to nirsModelPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function spmPathInput_Callback(~, ~, ~)
% hObject    handle to spmPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function spmPathInput_CreateFcn(hObject, ~, ~)
% hObject    handle to spmPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function frameSkipInput_Callback(~, ~, ~)
% hObject    handle to frameSkipInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function frameSkipInput_CreateFcn(hObject, ~, ~)
% hObject    handle to frameSkipInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function stickerMinGroupSizeInput_Callback(~, ~, ~)
% hObject    handle to stickerMinGroupSizeInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function stickerMinGroupSizeInput_CreateFcn(hObject, ~, ~)
% hObject    handle to stickerMinGroupSizeInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function radiusToStickerRatioInput_Callback(~, ~, ~)
% hObject    handle to radiusToStickerRatioInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function radiusToStickerRatioInput_CreateFcn(hObject, ~, ~)
% hObject    handle to radiusToStickerRatioInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function spmFNIRSPathInput_Callback(~, ~, ~)
% hObject    handle to spmFNIRSPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function spmFNIRSPathInput_CreateFcn(hObject, ~, ~)
% hObject    handle to spmFNIRSPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function stickerHSVPathInput_Callback(~, ~, ~)
% hObject    handle to stickerHSVPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function stickerHSVPathInput_CreateFcn(hObject, ~, ~)
% hObject    handle to stickerHSVPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function shimadzuFilePathInput_Callback(~, ~, ~)
% hObject    handle to shimadzuFilePathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function shimadzuFilePathInput_CreateFcn(hObject, ~, ~)
% hObject    handle to shimadzuFilePathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function vsfmPathInput_Callback(~, ~, ~)
% hObject    handle to vsfmPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function vsfmPathInput_CreateFcn(hObject, ~, ~)
% hObject    handle to vsfmPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function modelMeshPathInput_Callback(~, ~, ~)
% hObject    handle to modelMeshPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function modelMeshPathInput_CreateFcn(hObject, ~, ~)
% hObject    handle to modelMeshPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in browseModelMesh.
function browseModelMesh_Callback(~, ~, handles)
% hObject    handle to browseModelMesh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file, path, ~] = uigetfile('*.ply');
if file == 0
    return;
end
set(handles.modelMeshPathInput, 'String', fullfile(path, file));

function outputDirInput_Callback(~, ~, ~)
% hObject    handle to outputDirInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function outputDirInput_CreateFcn(hObject, ~, ~)
% hObject    handle to outputDirInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
