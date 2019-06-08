function [R] = spm_fnirs_spatialpreproc_ui(F)
% Transform optode/channel positions from subject space to MNI space 
% FORMAT [R] = spm_fnirs_spatialpreproc_ui(F) 
%
% F     name of files for spatial registration 
%
% R     structure array of optode/channel positions 
%
%--------------------------------------------------------------------------
% F{1,1}        (i) names of files including optode positions in subject
%                   space or MNI space 
%                   (ii) name of file including channel configuration 
% F{2,1}        name(s) of file(s) including fNIRS measurements paired with
%                   optode/channel positions. 
%__________________________________________________________________________
% Copyright (C) 2015 Wellcome Trust Centre for Neuroimaging

%
% $Id$

fprintf('--------------------------------------------------------- \n'); 
fprintf('Transform optode/channel positions from subject space to MNI space...\n'); 
fprintf('--------------------------------------------------------- \n'); 

if ~nargin, F = cell(3,1); end 

%--------------------------------------------------------------------------
spm_input('Transform channel positions from subject to MNI space:', 1, 'd');
fprintf('Read optode, channel and (reference) positions...\n'); 

if isempty(F{1,1}) 
    [F{1,1}, sts] = spm_select([1 3], 'any', 'Select files for spatial registration of fNIRS channels');
    if ~sts, R = []; return; end
end

if isempty(F{2,1}) 
    [F{2,1} sts] = spm_select([1 Inf], '^*.*\.mat$', 'Select fNIRS measurements (Y) to be paired with the channel');
    if ~sts, R = []; return; end
end

if size(F{1,1}, 1) == 1 && ~isempty(regexpi(F{1,1}, 'POS.mat'))
    sfname = F{1,1}; 
    load(sfname); 
    fprintf('Completed. \n\n');
    
else
    R = spm_fnirs_read_pos(F{1,1});
    
    noptd = R.s.ns+R.d.nd;
    indx_s = 1:R.s.ns; indx_d = R.s.ns+1:noptd;
    indx_ch = noptd+1:noptd+R.ch.nch;
    
    fprintf('Completed. \n\n');
    %--------------------------------------------------------------------------
    if isfield(R.ch, 'xyzS') % subject space
        fprintf('Transform subject space to MNI space using NFRI software...\n');
        xyzS = [R.s.xyzS R.d.xyzS R.ch.xyzS];
        xyzM = nfri_mni_estimation_spm(R.fname.ref, xyzS');
        
        % save MNI coordinates in structure of R
        s{1} = 'H'; s{2} = 'C';
        for i = 1:2
            R.s.(sprintf('xyz%s', s{i})) = xyzM.(sprintf('Other%s', s{i}))(indx_s,:)';
            R.d.(sprintf('xyz%s', s{i})) = xyzM.(sprintf('Other%s', s{i}))(indx_d,:)';
            R.ch.(sprintf('xyz%s', s{i})) = xyzM.(sprintf('Other%s', s{i}))(indx_ch,:)';
        end
        
        xyzM = xyzM.OtherC';
        
        fprintf('Completed. \n\n');
    else
        fprintf('Optode and channel positions in MNI space are identified. \n\n');
        xyzM = [R.s.xyzC R.d.xyzC R.ch.xyzC];
    end
    
    %--------------------------------------------------------------------------
    % project MNI coordinates onto the rendered spm brain
    
    fprintf('Project MNI coordinates onto the rendered brain...\n');
    load('render_mni_icbm152.mat');
    [xy] = spm_fnirs_xy_render(xyzM, rend);
    
    % xy on the rendered brain
    nv = size(xy, 2);
    for i = 1:nv
        R.s.xy{i} = xy{i}(:, indx_s);
        R.d.xy{i} = xy{i}(:, indx_d);
        R.ch.xy{i} = xy{i}(:, indx_ch);
        
        R.rend{i}.ren = rend{i}.ren;
        R.rend{i}.xyz = rend{i}.xyz;
    end
    
    fprintf('Completed. \n\n');
    
    %--------------------------------------------------------------------------
    % save results
    
    fprintf('Save POS.mat... \n');
    [sdir, name, ext] = fileparts(R.fname.optd);
    sfname = fullfile(sdir, 'POS.mat');
    R.fname.pos = sfname;
    if nargout == 0, save(R.fname.pos, 'R', spm_get_defaults('mat.format')); end
    fprintf('Completed. \n\n');
end

%--------------------------------------------------------------------------
% update header of Y to be paired with channels

fprintf('fNIRS measurements (NIRS.mat) are paired with channel positions (POS.mat)...\n');
nY = size(F{2,1}, 1);
for i = 1:nY
    load(deblank(F{2,1}(i,:)), 'P'); 
    P.fname.pos = sfname; 
    save(P.fname.nirs, 'P', '-append', spm_get_defaults('mat.format'));
end
fprintf('Completed. \n\n');

%--------------------------------------------------------------------------
% display spatial registration results 
spm_fnirs_viewer_ch(R); 

fprintf('--------------------------------------------------------- \n'); 
fprintf('%-40s: %30s\n','Completed.',spm('time')); 
fprintf('--------------------------------------------------------- \n'); 

