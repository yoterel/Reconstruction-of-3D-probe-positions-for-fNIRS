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

% Last Modified by GUIDE v2.5 10-Sep-2019 11:14:27

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


% --- Executes just before startWindow is made visible.
function startWindow_OpeningFcn(hObject, eventdata, handles, varargin)
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
function varargout = startWindow_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in submitBtn.
function submitBtn_Callback(hObject, eventdata, handles)
% hObject    handle to submitBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
app("Hello world");



function modelMeshInput_Callback(hObject, eventdata, handles)
% hObject    handle to modelMeshInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of modelMeshInput as text
%        str2double(get(hObject,'String')) returns contents of modelMeshInput as a double


% --- Executes during object creation, after setting all properties.
function modelMeshInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to modelMeshInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vsfmPath_Callback(hObject, eventdata, handles)
% hObject    handle to vsfmPath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vsfmPath as text
%        str2double(get(hObject,'String')) returns contents of vsfmPath as a double


% --- Executes during object creation, after setting all properties.
function vsfmPath_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vsfmPath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vidPathInput_Callback(hObject, eventdata, handles)
% hObject    handle to vidPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vidPathInput as text
%        str2double(get(hObject,'String')) returns contents of vidPathInput as a double


% --- Executes during object creation, after setting all properties.
function vidPathInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vidPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function mniModelPathInput_Callback(hObject, eventdata, handles)
% hObject    handle to mniModelPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mniModelPathInput as text
%        str2double(get(hObject,'String')) returns contents of mniModelPathInput as a double


% --- Executes during object creation, after setting all properties.
function mniModelPathInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mniModelPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function nirsModelPathInput_Callback(hObject, eventdata, handles)
% hObject    handle to nirsModelPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of nirsModelPathInput as text
%        str2double(get(hObject,'String')) returns contents of nirsModelPathInput as a double


% --- Executes during object creation, after setting all properties.
function nirsModelPathInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nirsModelPathInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function frameSkipInput_Callback(hObject, eventdata, handles)
% hObject    handle to frameSkipInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of frameSkipInput as text
%        str2double(get(hObject,'String')) returns contents of frameSkipInput as a double


% --- Executes during object creation, after setting all properties.
function frameSkipInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to frameSkipInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
