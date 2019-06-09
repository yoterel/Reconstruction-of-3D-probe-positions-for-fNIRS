%% Description
% Conversion of Shimadzu data (.txt) to HOMER2 readable format (.nirs)

% This script converts the Shimadzu data (.txt) to readable format (.nirs)
% for use with the HOMER2 NIRS processing package
% To use this script, the user must first create an .SD file which matches
% their Shimadzu probe layout using the SDgui function of Homer2 (or AtlasViewerGUI/Make Probe).
% Refer to Aasted et al., Neurophotonics, 2014 for a tutorial.

%% INPUT:
% User needs to have the following files in the current matlab directory:
% Shimadzu data: the acceptable format is .txt
% SD file: the SD structure with .SD format

%% OUTPUT:
% filename.nirs: contains d, aux, s, t, tIncMan,SD
% d:     intensity signal (#time points x #channels)
% aux:   auxiliary data is all zeros (#time points)
% s:     stimulus marker vector (1s at the beginning of each stimulus) Note
% that user needs to input homer with condition information as the
% Schimadzu text file does not provide condition specific stimulus onset information.
% t:     time vector
% SD:    probe layout
% tIncMan: Data that has been manually excluded. 0-excluded. 1-included (movement).
%        Vector same length as d. (#time points).

% LOG:
% created 04-14-2017, Created by Sahar Jahani

% TO DO:
%
function Shimadzu2nirsSingleFile(filename)
[pathName,prevName] = fileparts(filename);
fid = fopen(filename);
filelist = dir(strcat(pathName, filesep, "*.SD"));
load(strcat(pathName, filesep, filelist.name), '-mat'); % loading SD file
C = textscan(fid, '%s', 'delimiter', '\n');
C = [C{1,1}];
c=0;
% the HbO, HbR and HbT information
for i=37:(size(C,1))
    c=c+1;
    Data(c,:)=(str2num(C{i,:}));
end

dc = zeros(size(Data,1),3,(size(Data,2)-4)/3);
% Extracting the HbO, HbR, HbT, t and s information separately
dc(:,1,:) = Data(:,5:3:end-2);
dc(:,2,:) = Data(:,6:3:end-1);
dc(:,3,:) = Data(:,7:3:end);
t=Data(:,1); % time course vector
task = Data(:,2);
mark = Data(:,3);
conds = unique(task(task>0));
s = zeros(length(t),max(conds)); % stimmark vector
for condInd = 1:length(conds)
    cond = conds(condInd);
    s(task==cond,cond) = 1;
end

tIncMan=ones(size(t)); % tInc manually
aux=zeros(size(t)); % auxiliary data


% conversion from concentration to OD
procResult.dod = hmrConc2OD(dc,SD,[ones(1,length(SD.Lambda))*6]);
% conversion from OD to Intensity
d = exp(-procResult.dod*10^-4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ############# CORRECT CHANNEL LOCATION IN d ACCORDING TO THE SD FILE
% AND THE ORIGINAL CHANNEL LOCATIONS OBTAINED FROM TEXT FILE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
list = (C{33,:}); % get the channel information
list = regexprep(list,',',' '); % get rid of characters '(' &  ')' & ','
list = regexprep(list,'(','');
list = regexprep(list,')',' ');
list = str2num(list); % convert to number
list = reshape(list,2,size(dc,3))';
wavelength_info = [ones(1,size(list,1)) 2*ones(1,size(list,1)) 3*ones(1,size(list,1))]'; % add wavelength info
list = [list;list;list];
list = [list wavelength_info];

foo = zeros(size(d));
for i = 1:size(list,1);
    lst = find(list(:,1) == SD.MeasList(i,1) & list(:,2) == SD.MeasList(i,2) & list(:,3) == SD.MeasList(i,4));
    foo(:,i) = d(:,lst);
end
d = foo;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% get the name of the data file
fs = 1/(t(2)-t(1));
raw_dc = dc;
% saving data in .nirs format
save(strcat(pathName, filesep, prevName, ".nirs"), 'd', 'aux', 's', 't', 'tIncMan', 'SD', 'fs', ...
    'procResult', 'mark', 'raw_dc');

