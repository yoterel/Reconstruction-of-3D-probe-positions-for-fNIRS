function [R] = spm_fnirs_read_pos(fname)
% Read optode, reference positions, or channel configuration from text file
% FORMAT [R] = spm_fnirs_read_pos(fname) 
%
% fname     name of text files (.csv or .txt), including position information 
%
% R             structure array of optode/channel positions 
%
%__________________________________________________________________________
% Copyright (C) 2015 Wellcome Trust Centre for Neuroimaging

%
% $Id$

nfiles = size(fname, 1); 
for i = 1:nfiles 
    fid = fopen(deblank(fname(i,:))); 
    slabel = fgetl(fid); 
    
    if ~isempty(regexpi(slabel, 'Reference')) % reference positions 
        R.fname.ref = fname(i,:);
        
    elseif ~isempty(regexpi(slabel, 'Optode')) % optode positions 
        R.fname.optd = fname(i,:);
        
        ns = 0; nd = 0;
        label_s = []; label_d = [];
        while 1
            tline = fgetl(fid);
            if ~ischar(tline), break, end
            indx = find(tline == ','); tline(indx) = ' ';
            [nlabel, data] = strtok(tline);
            
            num = str2num(nlabel(2:end));
            if ~isempty(strfind(nlabel, 'T')) || ~isempty(strfind(nlabel, 'S')) % source position
                ns = ns + 1;
                xyz_s(:, ns) = str2num(data)';
                label_s = [label_s num];
                
            elseif ~isempty(strfind(nlabel, 'R')) || ~isempty(strfind(nlabel, 'D')) % detector position
                nd = nd + 1;
                xyz_d(:,nd) = str2num(data)';
                label_d = [label_d num];
                
            end
        end
        [R.s.label, indx_s] = sort(label_s);
        [R.d.label, indx_d] = sort(label_d);
        
        R.s.ns = ns;
        R.d.nd = nd;
        
        if ~isempty(regexpi(slabel, 'MNI')) % optode positions in MNI space 
            R.s.xyzC = xyz_s(:, indx_s); 
            R.d.xyzC = xyz_d(:, indx_d); 
        else % optode positions in subject space 
            R.s.xyzS = xyz_s(:, indx_s);
            R.d.xyzS = xyz_d(:, indx_d);
        end
    
    elseif ~isempty(regexpi(slabel, 'Ch')) % channel configuration which defines a pair of source and detector 
        R.fname.ch_sd = fname(i,:);
        
        nch = 0;
        while 1
            tline = fgetl(fid);
            if ~ischar(tline), break, end
            nch = nch + 1;
            indx = find(tline == ','); tline(indx) = ' ';
            
            ch_sd(nch, :) = str2num(tline);
        end
        R.ch_sd = ch_sd;
        R.ch.nch = nch;
    end
    fclose(fid);
end

%Calculate channel positions in subject or MNI space
R.ch.label = []; 
for i = 1:R.ch.nch
    indx_s = find(R.s.label == R.ch_sd(i, 2));
    indx_d = find(R.d.label == R.ch_sd(i, 3));
    try R.ch.xyzS(1:3, i) = (R.s.xyzS(1:3, indx_s) + R.d.xyzS(1:3, indx_d))./2; end
    try R.ch.xyzC(1:3, i) = (R.s.xyzC(1:3, indx_s) + R.d.xyzC(1:3, indx_d))./2; end
    R.ch.label = [R.ch.label R.ch_sd(i,1)]; 
end

R.ch.mask = ones(1, R.ch.nch); 



        
        