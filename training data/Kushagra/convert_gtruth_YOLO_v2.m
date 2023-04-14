function convert_gtruth_YOLO_v2(gname,folder)

%Convert Matlab ground truth data to YOLO format
% clear all
% close all


decim = 6; %Take every nth frame

%Load the data (select here what batch of labels to convert. Before running this, export the ground truth from the labelr app.)
%load gTruth_RIGHT_598.mat; %gTruth_LEFT_598.mat %gTruth_left.mat
load(gname);

%Folder names for output
%folder = 'right_598'; %'left';
mkdir(folder);
cd(folder);
%And subfolders
images_folder = 'images';
mkdir(images_folder);
cd(images_folder)
path_to_images = pwd;
cd ..
labels_folder = 'labels';
mkdir(labels_folder);
cd(labels_folder);
path_to_labels = pwd;
cd ..

%Open the video filename
v_filename = gTruth.DataSource.SourceName;
v = VideoReader('kush_01.mp4');

%Convert labels table to cell5
labels = table2cell(timetable2table(getfield(gTruth.ROILabelData, gTruth.DataSource.SignalName)));
label_definitions = fieldnames(getfield(gTruth.ROILabelData, gTruth.DataSource.SignalName));

numframes = eval(['size(gTruth.ROILabelData.' char(gTruth.DataSource.SignalName) ',1)']);
indices = 1:decim:numframes; %size(gTruth.ROILabelData.video_RIGHT_2022_08_27_17_28_01_313_Trim,1);

fnum = 1;
%Go through each frame and extract the figure and labels
for ll = 1:length(indices) %ii=1:size(gTruth.ROILabelData.video_LEFT_2022_08_27_17_28_01_313_Trim,1)
    ii = indices(ll)
%     %Video files
%     video_frame = read(v,ii);
%     imwrite(video_frame,strcat(path_to_images,'/', num2str(ll),'.png'));
    %Labels
    temp_labels = [];
    for jj=1:(length(label_definitions)-3) %Go throuh the different labels
        for kk=1:size(labels{ii,jj+1},1)
            %coordinates = labels{ii,jj+1}(1,kk).Position;
            coordinates = labels{ii,jj+1}(kk,:);
            coordinates_YOLO = [coordinates(1,1)+coordinates(1,3)/2 coordinates(1,2)-coordinates(1,4)/2 coordinates(1,3) coordinates(1,4)];
            temp_labels = [temp_labels; jj coordinates_YOLO];
        end
    end
    if ~isempty(temp_labels)
        %If frame contains labels, save the image and labels
        fileID = fopen(strcat(path_to_labels,'/', num2str(fnum),'.txt'),'w');
        fprintf(fileID,'%u %12.8f %12.8f %12.8f %12.8f\n',temp_labels');
        fclose(fileID);
        video_frame = read(v,ii);
        imwrite(video_frame,strcat(path_to_images,'/', num2str(fnum),'.png'));
        fnum = fnum + 1;
    end
end

%Write the metadata

fileID = fopen('metadata.txt','w');
for jj=1:(length(label_definitions)-3)
    fprintf(fileID,'%u %s\n',jj,label_definitions{jj,1});
end
fclose(fileID);
cd ..
