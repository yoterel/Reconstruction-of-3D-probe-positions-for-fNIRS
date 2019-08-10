function [candidates] = getStickerCandidates(handles, plyFilePath, stickerHSV)
%GETSTICKERCANDIDATES Summary of this function goes here
%   Detailed explanation goes here
setStatusText(handles, "Reading ply file");
mesh = plyread(plyFilePath);
setStatusText(handles, "Finished reading ply file, finding sticker candidate points");

vertices = [mesh.vertex.x mesh.vertex.y mesh.vertex.z];
verColors = [mesh.vertex.diffuse_red mesh.vertex.diffuse_green mesh.vertex.diffuse_blue];
verColorsHSV = rgb2hsv(double(verColors)/255);
trgHue = stickerHSV(1);
candidates = vertices(abs(verColorsHSV(:,1) - trgHue) < 0.15 & ... 
    ((verColorsHSV(:, 2) > 0.2 & verColorsHSV(:, 3) > 0.2) | (verColorsHSV(:,2) > 0.3 & verColorsHSV(:, 3) > 0.1)), :);
setStatusText(handles, "Found sticker candidate vertices");
% Plot the candidates
plot3(candidates(:,1), candidates(:,2), candidates(:,3), '.');
end

