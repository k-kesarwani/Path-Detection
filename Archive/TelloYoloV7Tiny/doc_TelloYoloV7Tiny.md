# TelloYoloV7.py Documentation
## Introduction
This code implements the YoloV7-Tiny object detection model on the Tello drone. It uses OpenCV, DJI TelloPy, and CVZone libraries for image processing, drone control, and drawing bounding boxes and labels around detected objects.

## Code description
- Import the necessary libraries:
    - ``cv2`` for image processing
    - ``tello`` from ``djitellopy`` for drone control
    - ``cvzone`` for drawing bounding boxes and labels around detected objects.
- Set the confident threshold and non-maximum suppression threshold for object detection
````
thres = 0.50
nmsThres = 0.2
````
- Read the class names from a file and create a list of class names and print them
````
classNames = []
classFile = 'coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
print(classNames)
````
- Load the YoloV7-Tiny model from the downloaded weights and configuration file
````
configPath = 'yolov7-tiny.cfg'
weightsPath = 'yolov7-tiny.weights'
net = cv2.dnn_DetectionModel(weightsPath, configPath)
````
- Set the input size, input scale, and input swapRB for the model
````
net.setInputSize(416, 416)
net.setInputScale(1.0 / 255)
net.setInputSwapRB(True)
````
- Initialize the Tello drone object and establish a connection
````
me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamoff()
me.streamon()
me.takeoff()
````
- Enter a loop that captures frames from the drone's video stream and detects objects in the frames using the YoloV7-Tiny model
````
while True:
    img = me.get_frame_read().frame
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nmsThres)
    try:
        for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
            cvzone.cornerRect(img, box)
            cv2.putText(img, f'{classNames[classId]} {conf:.2f}',
                (box[0] + 10, box[1] + 30), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                1, (0, 255, 0), 2)
    except:
        pass
    me.send_rc_control(0, 0, 0, 0)
    cv2.imshow("Image", img)
    cv2.waitKey(1)
````
- If a keyboard interrupt is detected, land the drone, close the OpenCV window, turn off the video stream, and disconnect from the drone
````
except KeyboardInterrupt:
    print("KeyboardInterrupt: landing the drone and exiting...")
    me.land()
    cv2.destroyAllWindows()
    me.streamoff()
    me.end()
````
