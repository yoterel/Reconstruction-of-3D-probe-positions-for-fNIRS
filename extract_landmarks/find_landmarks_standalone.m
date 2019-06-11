function [landmarks] = find_landmarks_standalone(file_path)
    %replace with model file location
    model_file=fullfile('68_face_landmarks_model.dat');
    %replace with python script location
    python_file=fullfile('landmarks_standalone.py');
    %notice a "skip" parameter can be passed to the script if needed
    %IMPORTANT: open the script with -h in python to discover all options
    commandStr = sprintf('python "%s" --image_file "%s" "%s"', python_file, file_path, model_file);
    [status, commandOut] = system(commandStr,'-echo');
    if (status)
        X = ['python parser failed on file ',xmlFile];
        disp(X);
        disp(commandOut);
        return;
    end
    %the following processing converts the raw string output fom string to
    %matlab cell array
    startIndex=regexp(commandOut,'\[','once');
    landmarks=commandOut(startIndex:end);
    landmarks(isspace(landmarks))=[];
    landmarks=strip(landmarks);
    landmarks=landmarks(2:end-1);
    output={};
    out=[];
    a = regexp(landmarks,'\[{1,2}[\d,]*\]{1,2}','match');
    for i=1:1:length(a)
        if strcmp(a{i},'[]')
            output{end+1}=[];
            out=[];
        else
            if strfind(a{i},']]')
                b=regexp(a{i},'\d*','match');
                out=[out; cellfun(@str2num,b)];
                output{end+1}=out;
                out=[];
            else
                b=regexp(a{i},'\d*','match');
                out=[out; cellfun(@str2num,b)];
            end                
        end
    end
    landmarks=output;
    %we want to create a new point for every triangle found (make it a
    %polygon), this loop performs this.
    for i=1:1:length(landmarks)
        [m,n]=size(landmarks{i});
        for j=1:1:m
            x=[landmarks{i}(j,1), landmarks{i}(j,3), landmarks{i}(j,5)];
            y=[landmarks{i}(j,2), landmarks{i}(j,4), landmarks{i}(j,6)];
            p=mirrorPoint(x,y); %reflect nose point about [left,right] points
            tempx=x(3);
            tempy=y(3);
            landmarks{i}(j,5)=p(1);
            landmarks{i}(j,6)=p(2);
            landmarks{i}(j,7)=tempx;
            landmarks{i}(j,8)=tempy;
        end
    end
end