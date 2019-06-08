function makeListAndConnection(pathname, framerate, skip_frames, imgResultFilePrefix, ...
    connectionsFileName)
%makeListAndConnection creates metadata text files required by VisualSFM
%for mesh reconstruction
denseConnectionRadius = round(framerate / (skip_frames+1));
connectionsFilePath = strcat(pathname, filesep, connectionsFileName);
fprintf("Creating output file: %s\n", connectionsFilePath);
connectfid = fopen(connectionsFilePath, 'wt');
filelist = dir(strcat(pathname ,filesep, imgResultFilePrefix, '*.jpg'));
listfid = fopen(strcat(pathname, filesep,'list.txt'), 'wt');
for ii = 1:length(filelist)
    fprintf(listfid,[filelist(ii).name,'\n']);
    for jj = 1:denseConnectionRadius
        fprintf(connectfid, '%s\n', [filelist(ii).name,' ',filelist(mod(ii+jj-1,length(filelist))+1).name]);
    end
end
fclose(listfid);
fclose(connectfid);
