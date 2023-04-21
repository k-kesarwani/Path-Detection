import cv2
from djitellopy import tello
import cvzone

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

#me.takeoff()

# Define the video writer to save the first 30 seconds of the video stream
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 20, (960, 720))

start_time = cv2.getTickCount()

try:
    while True:
        # Get the current frame from the Tello drone's video stream
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

        # Write the current frame to the video file
        out.write(img)

        # Check if 30 seconds have passed
        elapsed_time = (cv2.getTickCount() - start_time) / cv2.getTickFrequency()
        if elapsed_time > 30:
            break

        cv2.waitKey(1)

except KeyboardInterrupt:
    print("KeyboardInterrupt: landing the drone and exiting...")
    #me.land()
    cv2.destroyAllWindows()
    me.streamoff()
    me.end()  # Disconnect from the drone

# Release the video writer and destroy the OpenCV windows
out.release()
cv2.destroyAllWindows()