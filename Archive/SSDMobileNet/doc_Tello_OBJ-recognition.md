# Introduction
This script uses the DJI Tello drone with a pre-trained object detection model to detect objects in real-time. The script utilizes the OpenCV and CVzone libraries for object detection and drawing on images.

## Import Libraries
````
import cv2
from djitellopy import tello
import cvzone
````
The script uses OpenCV, djitellopy and cvzone libraries which must be installed before running the script.

## Set thresholds
````
thres = 0.50
nmsThres = 0.2
````
These variables are used to set the detection threshold and non-maximum suppression threshold. They determine the minimum confidence level required for an object to be detected and how many overlapping bounding boxes should be removed.

## Read Class Names
````
classNames = []
classFile = 'ss.names'
with open(classFile, 'rt') as f:
    classNames = f.read().split('\n')
````
This code reads a text file containing the names of the objects that can be detected by the model. The file contains one class name per line.

## Load Model
````
configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = "frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)
````
The script loads the pre-trained model using the weights and configuration files. It also sets the input size, scale, mean and color channel order for the model.

## Connect to Tello Drone
````
me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamoff()
me.streamon()
me.takeoff()
````
This code initializes the Tello drone object and connects to the drone's WiFi network. The script also turns off the stream and turns it back on and takes off the drone.

## Object detection loop
````
while True:
    img = me.get_frame_read().frame
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nmsThres)
    try:
        for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
            cvzone.cornerRect(img, box)
            cv2.putText(img, f'{classNames[classId - 1].upper()} {round(conf * 100, 2)}',
                        (box[0] + 10, box[1] + 30), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                        1, (0, 255, 0), 2)
    except:
        pass

    me.send_rc_control(0, 0, 0, 0)

    cv2.imshow("Image", img)
    cv2.waitKey(1)
````
This code runs a loop that continuously captures frames from the drone's camera and detects objects in each frame using the pre-trained model. The detected objects are drawn on the image and displayed in a window. The script also sends zero commands to the drone to keep it stable in the air.