#   File:                       TelloYoloV7.py
#   Author:                     Paavo Mäkelä
#   Description:                Implementing YoloV7-Tiny on tello drone
#   Weights from:               https://github.com/AlexeyAB/darknet/releases/download/yolov4/yolov7-tiny.weights
#   CFG from:                   https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov7-tiny.cfg                
#   Official YoloV7 Repo:       https://github.com/WongKinYiu/yolov7

import cv2                              #pip install opencv-python
from djitellopy import tello            #pip install djitellopy
import cvzone                           #pip install cvzone

thres = 0.50
nmsThres = 0.2

classNames = []
classFile = 'coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

print(classNames)
configPath = 'yolov7-tiny.cfg'
weightsPath = 'yolov7-tiny.weights'

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(416, 416)
net.setInputScale(1.0 / 255)
net.setInputSwapRB(True)

me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamoff()
me.streamon()

me.takeoff()

while True:
    img = me.get_frame_read().frame
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nmsThres)
    try:
        for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
            cvzone.cornerRect(img, box)
            cv2.putText(img, f'{classNames[classId]} {conf:.2f}',
                        (box[0] + 10, box[1] + 30), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                        1, (0, 255, 0), 2)
            #if classNames[classId] == 'person':
                #me.land()
    except:
        pass

    me.send_rc_control(0, 0, 0, 0)

    cv2.imshow("Image", img)
    cv2.waitKey(1)