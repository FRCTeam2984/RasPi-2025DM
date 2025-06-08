# Import code lib for various commands used in the code
from collections import deque

import numpy as np
import apriltag
import cv2

import math
from networktables import NetworkTables

tagsize = 0.152
fx = 484.34625874
fy = 483.97948992
cx = 304.89078752
cy = 236.58729305
# Network tables
# change server IP for the robot when added to robot, rn the server IP i s just the local address for testing
NetworkTables.initialize(server="10.29.84.2")
sd = NetworkTables.getTable("SmartDashboard")

# Begin writing varibles that do not change throughout varibles

capture = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
hres = 1280
vres = 720
capture.set(3, hres)
capture.set(4, vres)
#capture_dim = [capture.get(cv2.CAP_PROP_FRAME_WIDTH), capture.get(cv2.CAP_PROP_FRAME_HEIGHT)]
#capture.set(cv2.CAP_PROP_FRAME_WIDTH, capture_dim[0])
#capture.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_dim[1])
# capture.set(15, -8)


def apriltag_detection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('Gray Frame', gray)
    apriltag.DetectorOptions(families="tag36h11", nthreads="4")
    detector = apriltag.Detector()
    detections = detector.detect(gray)
    if (detections):
        if (detections[0].tag_id <= 16):
            print(detections)
            

            cameraparams = (fx,fy,cx,cy)
            pose = detector.detection_pose(detections[0], cameraparams, tagsize)
            for tag in detections:
                
                print(pose)
                print(pose[0][2][3])
                print(tag.center[0] - (hres/2))
                print(tag.center[1] - (vres/2))
                print(tag.tag_id)
                centertag = str(tag.center)
                print(type(tag.center))
                print(type(centertag))
                centertag.replace('[', '')
                centertag.replace(']', '')
                print(centertag)
                tagcentery = tag.center[1] - 240
                tagcenterx = tag.center[0] - 320
                sd.putValue("tagread", tag.tag_id)
                sd.putValue("Distance", pose[0][2][3])
                sd.putValue("tagcentery", tagcentery)
                sd.putValue("tagcenterx", tagcenterx)
                sd.putBoolean("tagdetect", True)
        else:
          sd.putBoolean("tagdetect", False)
    else:
      sd.putBoolean("tagdetect", False)
    #cv2.waitKey(1)
    
    


# Begin repeated capture of video
while True:
    ret, frame = capture.read()
    apriltag_detection(frame)
    cv2.imshow('Reg Frame', frame)
    #
    
    cv2.waitKey(1)

