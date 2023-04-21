#   File:                       TelloYoloV7.py
#   Author:                     Paavo Mäkelä
#   Description:                Implementing YoloV7-Tiny on tello drone
#   Weights from:               https://github.com/AlexeyAB/darknet/releases/download/yolov4/yolov7-tiny.weights
#   CFG from:                   https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov7-tiny.cfg                
#   Official YoloV7 Repo:       https://github.com/WongKinYiu/yolov7


#Import the necessary libraries
import cv2                              #pip install opencv-python
from djitellopy import tello            #pip install djitellopy
import cvzone                           #pip install cvzone

# Set the confident threshold and non-maximum supression threshold for object detection
thres = 0.50
nmsThres = 0.2

# Read the class names from a file and create a list of class names and print them
classNames = []
classFile = 'coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
print(classNames)

# Load the YoloV7-Tiny model from the downloaded weights and configuration file
configPath = 'yolov7-tiny.cfg'
weightsPath = 'yolov7-tiny.weights'
net = cv2.dnn_DetectionModel(weightsPath, configPath)

# Set the input size, input scale, and input swapRB for the model
net.setInputSize(416, 416)
net.setInputScale(1.0 / 255)
net.setInputSwapRB(True)

# Initialize the Tello drone object and establish a connection
me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamoff()
me.streamon()
me.takeoff()

try:
    # Enter a loop that captures frames from the drone's video stream and detects objects in the frames using the YoloV7-Tiny model
    while True:
        # Get the latest frame from the drone's video stream and Detect objects in the frame
        img = me.get_frame_read().frame
        classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nmsThres)

        # Draw rectangles around detected objects and label them with the class name and confidence score
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
        # Display the annotated frame on the screen
        me.send_rc_control(0, 0, 0, 0)
        cv2.imshow("Image", img)
        cv2.waitKey(1)

# If a keyboard interrupt is detected, land the drone, close the OpenCV window, turn off the video stream, and disconnect from the drone
except KeyboardInterrupt:
    print("KeyboardInterrupt: landing the drone and exiting...")
    me.land()
    cv2.destroyAllWindows()
    me.streamoff()
    me.end()  # Disconnect from the drone