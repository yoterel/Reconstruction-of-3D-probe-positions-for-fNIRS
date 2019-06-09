function varargout = spm_fnirs_viewer_ch(varargin)
% GUI for (i) visualizing optode/channel positions on the surface of
% rendered brain, (ii) define set of channels of interest 
%
%__________________________________________________________________________
% Copyright (C) 2015 Wellcome Trust Centre for Neuroimaging

% 
% $Id$

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @spm_fnirs_viewer_ch_OpeningFcn, ...
                   'gui_OutputFcn',  @spm_fnirs_viewer_ch_OutputFcn, ...
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


% --- Executes just before spm_fnirs_viewer_ch is made visible.
function spm_fnirs_viewer_ch_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;

% default variables 
R= varargin{1,1}; handles.R = R; 
mask = R.ch.mask; handles.mask = mask; 

% set slider ui 
nsteps = 6; 
view = 2; % default view of brain (dorsal view) 
set(handles.slider_view, 'sliderstep', [1/(nsteps-1), 1/(nsteps-1)], 'max', nsteps, 'min', 1, 'value', view);

%------------------------------------------------------------------------------
% display channel and optodes positions on the rendered brain 

cmap = load('Split.mat'); 
brain = R.rend{view}.ren; % dorsal view 

axes(handles.axes_image); 
imagesc(brain, [0 2]); 
colormap(cmap.split); 
axis image
axis off 

%------------------------------------------------------------------------------
% channels 

% set font and background colors 
bcolor{1} = [0 0 0]; fcolor{1} = [1 0 0]; % non-ROI 
bcolor{2} = [1 1 1]; fcolor{2} = [1 0 0]; % ROI

indx = find(sum(R.ch.xy{view}) ~= 0); 
nch = length(indx);

for i = 1:nch 
    r = R.ch.xy{view}(1,indx(i));
    c = R.ch.xy{view}(2,indx(i)); 
    
    ch = R.ch.label(indx(i)); 
    text(c, r, num2str(ch), 'color', fcolor{mask(indx(i))+1}, 'FontWeight', 'bold', 'FontSize', 7, 'HorizontalAlignment', 'center', 'BackgroundColor', bcolor{mask(indx(i))+1}, 'Margin', 0.5); 
end
    
%------------------------------------------------------------------------------
% display channels in ROI and nonROI on listbox 
indx_roi = find(mask == 1); 
indx_nroi = find(mask == 0); 

str_list{1,1} = ['Chs in ROI: ' num2str(R.ch.label(indx_roi))]; 
str_list{2,1} = ['Chs in nonROI: ' num2str(R.ch.label(indx_nroi))]; 
set(handles.listbox_chs, 'value', 1, 'string', str_list); 

%------------------------------------------------------------------------------
% sources 
indx = find(sum(R.s.xy{view}) ~= 0); 
ns = length(indx);
for i = 1:ns
    r = R.s.xy{view}(1,indx(i));
    c = R.s.xy{view}(2,indx(i)); 
    text(c, r, 'o', 'color', 'b', 'FontWeight', 'bold', 'FontSize', 10, 'HorizontalAlignment', 'center'); 
end

indx = find(sum(R.d.xy{view}) ~= 0); 
nd = length(indx);
for i = 1:nd
    r = R.d.xy{view}(1,indx(i));
    c = R.d.xy{view}(2,indx(i)); 
    text(c, r, 'x', 'color', 'g', 'FontWeight', 'bold', 'FontSize', 10,  'HorizontalAlignment', 'center');
end

guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = spm_fnirs_viewer_ch_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;


% --- Executes on slider movement.
function slider_view_Callback(hObject, eventdata, handles)

R = handles.R; mask = handles.mask; 

cmap = load('Split.mat');
view = round(get(handles.slider_view, 'value')); set(handles.slider_view, 'value', view); 
brain = R.rend{view}.ren; % dorsal view 

% set font and background colors 
bcolor{1} = [0 0 0]; fcolor{1} = [1 0 0]; % non-ROI 
bcolor{2} = [1 1 1]; fcolor{2} = [1 0 0]; % ROI

axes(handles.axes_image); 
imagesc(brain, [0 2]); 
colormap(cmap.split); 
axis image
axis off 

% channels 
indx = find(sum(R.ch.xy{view}) ~= 0); 
nch = length(indx);

for i = 1:nch 
    r = R.ch.xy{view}(1,indx(i));
    c = R.ch.xy{view}(2,indx(i)); 
    
    ch = R.ch.label(indx(i)); 
    text(c, r, num2str(ch), 'color', fcolor{mask(indx(i))+1}, 'FontWeight', 'bold', 'FontSize', 7, 'HorizontalAlignment', 'center', 'BackgroundColor', bcolor{mask(indx(i))+1}, 'Margin', 0.5); 
end
    
% sources 
indx = find(sum(R.s.xy{view}) ~= 0); 
ns = length(indx);
for i = 1:ns
    r = R.s.xy{view}(1,indx(i));
    c = R.s.xy{view}(2,indx(i)); 
    text(c, r, 'o', 'color', 'b', 'FontWeight', 'bold', 'FontSize', 10, 'HorizontalAlignment', 'center'); 
end

indx = find(sum(R.d.xy{view}) ~= 0); 
nd = length(indx);
for i = 1:nd
    r = R.d.xy{view}(1,indx(i));
    c = R.d.xy{view}(2,indx(i)); 
    text(c, r, 'x', 'color', 'g', 'FontWeight', 'bold', 'FontSize', 10,  'HorizontalAlignment', 'center');
end


% --- Executes during object creation, after setting all properties.
function slider_view_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on selection change in listbox_chs.
function listbox_chs_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function listbox_chs_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in push_specify_ROI.
function push_specify_ROI_Callback(hObject, eventdata, handles)
%------------------------------------------------------------------------------
% obtain information of current figure
R = handles.R; 
mask = handles.mask; 

view = round(get(handles.slider_view, 'value')); set(handles.slider_view, 'value', view); 
brain = R.rend{view}.ren; % dorsal view 

indx_ch = find(sum(R.ch.xy{view}) ~= 0); 
nch = length(indx_ch);

%------------------------------------------------------------------------------
% specify set of channels 
brain_ch = zeros(size(brain)); 
for i = 1:nch 
    r = R.ch.xy{view}(1,indx_ch(i));
    c = R.ch.xy{view}(2,indx_ch(i)); 
    brain_ch(r, c) = indx_ch(i); 
end

mask_ch = roipoly; 
set_ch = inputdlg('Specify set of channels (ROI:1, non-ROI:0)'); if isempty(set_ch), return; end 
set_ch = str2num(set_ch{1,1}); 
if set_ch ~= 0, set_ch = 1; end 

brain_ch = brain_ch .* mask_ch; 
indx = find(brain_ch ~= 0); indx_set = brain_ch(indx); 

% update set of channels 
mask(indx_set) = set_ch; 

%------------------------------------------------------------------------------
% display channels on the rendered brain using updated mask information 
cmap = load('Split.mat');

% set font and background colors 
bcolor{1} = [0 0 0]; fcolor{1} = [1 0 0]; % non-ROI 
bcolor{2} = [1 1 1]; fcolor{2} = [1 0 0]; % ROI

axes(handles.axes_image); 
imagesc(brain, [0 2]); 
colormap(cmap.split); 
axis image
axis off 

% channels 
for i = 1:nch 
    r = R.ch.xy{view}(1,indx_ch(i));
    c = R.ch.xy{view}(2,indx_ch(i)); 
    ch = R.ch.label(indx_ch(i));
    text(c, r, num2str(ch), 'color', fcolor{mask(indx_ch(i))+1}, 'FontWeight', 'bold', 'FontSize', 7, 'HorizontalAlignment', 'center', 'BackgroundColor', bcolor{mask(indx_ch(i))+1}, 'Margin', 0.5); 
end
    
% sources 
indx = find(sum(R.s.xy{view}) ~= 0); 
ns = length(indx);
for i = 1:ns
    r = R.s.xy{view}(1,indx(i));
    c = R.s.xy{view}(2,indx(i)); 
    text(c, r, 'o', 'color', 'b', 'FontWeight', 'bold', 'FontSize', 10, 'HorizontalAlignment', 'center'); 
end

indx = find(sum(R.d.xy{view}) ~= 0); 
nd = length(indx);
for i = 1:nd
    r = R.d.xy{view}(1,indx(i));
    c = R.d.xy{view}(2,indx(i)); 
    text(c, r, 'x', 'color', 'g', 'FontWeight', 'bold', 'FontSize', 10,  'HorizontalAlignment', 'center');
end

%------------------------------------------------------------------------------
% display channels in ROI and nonROI on the listbox 
indx_roi = find(mask == 1); 
indx_nroi = find(mask == 0); 

str_list{1,1} = ['Chs in ROI: ' num2str(R.ch.label(indx_roi))]; 
str_list{2,1} = ['Chs in nonROI: ' num2str(R.ch.label(indx_nroi))]; 
set(handles.listbox_chs, 'value', 1, 'string', str_list); 

handles.R = R;
handles.mask = mask; 

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in push_update.
function push_update_Callback(hObject, eventdata, handles)
% update channel configuration on the header of measurements 
fprintf('Mask for channels of interest is updated and saved in POS.mat. \n\n');

R = handles.R; 
R.ch.mask = handles.mask; 

save(R.fname.pos, 'R', spm_get_defaults('mat.format')); 
