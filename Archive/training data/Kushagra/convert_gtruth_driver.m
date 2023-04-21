clear all
close all

first =1;
last = 32;
nc =0;

while first < last
    
    if first == 22
        first = first+1;
    end
    a= 'vid_' + string(first);
    if first < 10
        a = 'vid_' + string(0) + string(first);
    end
    b = a + '.mat';
    convert_gtruth_YOLO_v2_videolabeler(b, 'test2', nc);
    first = first +1;
    load('nc.mat','nc');
end