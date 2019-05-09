function [segmentedFrame] = predict(frame,net)
    resizedFrame=imresize(frame,[360 480]);
    resizedSegmentedFrame=semanticseg(resizedFrame, net);
    %cat_segmentedFrame=imresize(resizedSegmentedFrame,[1080 1920]);
    segmentedFrame=zeros([360 480],'uint8');
    segmentedFrame(resizedSegmentedFrame=="Cap")=1;
    segmentedFrame=imresize(segmentedFrame,[1080 1920]);
end