function [xy] = spm_fnirs_xy_render(xyz, rend, th_dist)
% Find xy coordinates on surface of rendered brain, which corresponds to 
% three-dimensional MNI coordinates 
% FORMAT [xy] = spm_fnirs_xy_render(xyz, rend, th_dist) 
%
% xyz       MNI coordinates [3 x # sample positions] 
% rend      structure array of surface of rendered brain 
%              note: 'rend' is generated using spm_surf.m function 
% th_dist   threshold to find xy positions corresponding to MNI coordinates
%
% xy         xy coordinates on surface of rendered brain
%__________________________________________________________________________
% Copyright (C) 2015 Wellcome Trust Centre for Neuroimaging

%
% $Id$

if nargin < 3, th_dist = 8; end 
    
nview = size(rend, 2); 
n = size(xyz, 2); 

for i = 1:nview % number of views
    xy{i} = zeros(2, n); 
    for j = 1:n 
        point = xyz(:, j); 
        nvox_r = size(rend{i}.xyz, 2); 
        s = size(rend{i}.ren); 
        
        dist = rend{i}.xyz(1:3, :) - point * ones(1, nvox_r); 
        dist = sqrt(sum(dist.^2)); 
        
        indx = find(dist == min(dist)); 
        if dist(indx) < th_dist 
            [xy{i}(1,j) xy{i}(2, j)] = ind2sub(s, indx); 
        end
    end
end
