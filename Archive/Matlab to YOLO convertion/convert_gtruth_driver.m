clear all
close all

%convert_gtruth_YOLO_v2('gtruth_0067','0067')
% convert_gtruth_YOLO_v2('gtruth_0064','0064') <-not converted yet
%convert_gtruth_YOLO_v2('gtruth_0115','0115')
%convert_gtruth_YOLO_v2('gtruth_0125','0125')
%convert_gtruth_YOLO_v2('gtruth_0126','0126')
% convert_gtruth_YOLO_v2('gtruth_0141','0141') <-not coverted yet

% convert_gtruth_YOLO_v2('dji_0065_session','0065')
% convert_gtruth_YOLO_v2('dji_0068_session','0068')
% convert_gtruth_YOLO_v2('dji_0098_session','0098')
% convert_gtruth_YOLO_v2('dji_0113_session','0113')
% convert_gtruth_YOLO_v2('dji_0123_session','0123')
% convert_gtruth_YOLO_v2('dji_0137_session','0137')
% convert_gtruth_YOLO_v2('dji_0138_session','0138')
% convert_gtruth_YOLO_v2('dji_0139','0139')
% convert_gtruth_YOLO_v2('dji_0142_session','0142')
% convert_gtruth_YOLO_v2('dji_0146_session','0146')
% convert_gtruth_YOLO_v2('dji_0148_session','0148')
% convert_gtruth_YOLO_v2('dji_0149','0149')
% convert_gtruth_YOLO_v2('dji_0154','0154')
% convert_gtruth_YOLO_v2('dji_0155','0155')

% convert_gtruth_YOLO_v2_videolabeler('gtruth_0099','0099') 
% convert_gtruth_YOLO_v2_videolabeler('gtruth_0104','0104')
% convert_gtruth_YOLO_v2_videolabeler('gtruth_0114','0114')
% convert_gtruth_YOLO_v2_videolabeler('gtruth_0143','0143')
% Variables to change the amount of videos to convert
first = 1;
last = 2;
% Variable to contain the count of images
nc = 0;
% While loop to make the convertion of multiple .mat files automatic
while first <= last
    % Name of the file
    a ='J'+string(first)
    % Makes it possible to convert multiple files with name J01 to J09
    if first < 10
        a ='J'+string('0')+string(first)
    end
    b = a+'.mat'
    % Runs different code to convert the .mat file
    convert_gtruth_YOLO_v2_videolabeler(b,'test2',nc);
    first=first+1;
    % loads the image count of the latest run
    load('nc.mat','nc')
end