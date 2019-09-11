function [bestReg, transformedCapHead, bestScale] = findHeadTransformation(capHead, modelHead)
%FINDHEADTRANSFORMATION Finds the best scaling, rotation and translation
% parameters for fitting the current cap head stars to the model stars.
% Performs grid search of axis-aligned scaling, using only the head points,
% and realign at each step.
scalespace = logspace(log10(0.5),log10(2),20);
bestD = inf;
for scX = scalespace
    for scY = scalespace
        for scZ = scalespace
            testCapHead = capHead.*[scX,scY,scZ]; % implicit expansion
            testReg = absor(testCapHead', modelHead');
            testCapHead = applyRegParams(testCapHead, testReg);
            testCapHead = testCapHead(:,1:3);
            % TODO: maybe not needed, need to use the labels
            [~,d] = knnsearch(modelHead, testCapHead); 
            %if sum(d) < bestD && (length(idx) == length(unique(idx)) || size(capStars,1) > size(existStars,1))
            %if ( cost < bestD )
            cost = sum(d.^2);
            if cost < bestD
                bestReg = testReg;
                transformedCapHead = testCapHead;
                bestScale = [scX,scY,scZ];
                bestD = cost;
            end
        end
    end
end
end