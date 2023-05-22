# Recording of Drone Object Detection using YOLOv7-tiny
This is a Python script that uses YOLOv7-tiny object detection to detect objects in real-time using a DJI Tello drone's video stream. The script utilizes OpenCV, DJITelloPy, and cvzone libraries.

## Dependencies
- cv2 (OpenCV)
- djitellopy
- cvzone

## Code documentation
````
import cv2
from djitellopy import tello
import cvzone
````
Importing necessary libraries - OpenCV, DJITelloPy, and cvzone
````
thres = 0.50
nmsThres = 0.2
````
Set threshold values for object detection - ``thres`` and ``nmsThres``.
````
classNames = []
classFile = 'coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

print(classNames)
````
Read the object class names from ``coco.names`` file and store it in ``classNames``.
````
configPath = 'yolov7-tiny.cfg'
weightsPath = 'yolov7-tiny.weights'

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(416, 416)
net.setInputScale(1.0 / 255)
net.setInputSwapRB(True)
````
Load YOLOv7-tiny model with the corresponding configuration and weight files. Set input size, scale and swapRB.
````
me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamoff()
me.streamon()
````
Connect to the Tello drone and print the battery level. Turn off the drone's video stream and then turn it back on.
````
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 20, (960, 720))
````
Define the video writer object to save the first 30 seconds of the video stream.
````
start_time = cv2.getTickCount()

try:
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

        out.write(img)

        elapsed_time = (cv2.getTickCount() - start_time) / cv2.getTickFrequency()
        if elapsed_time > 30:
            break

        cv2.waitKey(1)

except KeyboardInterrupt:
    print("KeyboardInterrupt: landing the drone and exiting...")
    cv2.destroyAllWindows()
    me.streamoff()
    me.end()
out.release()
cv2.destroyAllWindows()
````
In a continuous loop, the script reads the current frame from the drone's video stream, detects objects using YOLOv7-tiny object detection model, draws bounding boxes around the detected objects using cvzone.cornerRect function, and puts text on the image indicating the class name and confidence score of each object.