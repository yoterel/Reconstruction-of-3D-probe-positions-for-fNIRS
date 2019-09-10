function candidatesPc = getStickerCandidates(pc, stickerHSV)
%GETSTICKERCANDIDATES 
verColorsHSV = rgb2hsv(double(pc.Color) / 255);
trgHue = stickerHSV(1);
mask = abs(verColorsHSV(:,1) - trgHue) < 0.1 & ... 
    ((verColorsHSV(:, 2) > 0.2 & verColorsHSV(:, 3) > 0.2) | ...
        (verColorsHSV(:,2) > 0.3 & verColorsHSV(:, 3) > 0.1));
candidatesPc = filterPcPoints(pc, mask);
end

