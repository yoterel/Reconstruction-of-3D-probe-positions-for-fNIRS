function makeListAndConnection(pathname, framerate,skip_frames, imgResultFilePrefix, connectionsFilePath)
%sparseConnectionRadius = 240;
denseConnectionRadius = round(framerate / (skip_frames+1));
connectfid = fopen([pathname,filesep, connectionsFilePath],'wt');
filelist = dir([pathname,filesep, strcat(imgResultFilePrefix, '*.jpg')]);
listfid = fopen([pathname,filesep,'list.txt'],'wt');
for ii = 1:length(filelist)
    fprintf(listfid,[filelist(ii).name,'\n']);
    for jj = 1:denseConnectionRadius
        fprintf(connectfid,'%s\n',[filelist(ii).name,' ',filelist(mod(ii+jj-1,length(filelist))+1).name]);
    end
end
fclose(listfid);
fclose(connectfid);
