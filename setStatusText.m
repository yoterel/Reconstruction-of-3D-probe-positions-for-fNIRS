function setStatusText(handles, text, varargin)
set(handles.status, 'String', sprintf(text, varargin{:}));
drawnow;
end