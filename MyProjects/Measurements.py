#!/usr/bin/env python3
import json
from cmath import tan
import cv2
import time
import numpy as np
import math
from pupil_apriltags import Detector
from Constants import VisionConstants
import argparse
import ipaddress
import struct

class tagDG:
    frameCount = 0
    headerFormat = "HBB"
    tagFormat = "BbH"
    def __int__(self):
        self.tags = []
        self.frameID = tagDG.frameCount
        tagDG.frameCount += 1

    def pack(self):
        pass

    @staticmethod
    def unpack(dgramBytes):
        pass

    def addTag(self, tagID, distanceCM, angleDegrees):
        self.tags.append((int(tagID) &0xff, int(distanceCM) &0xffff, int(angleDegrees) &0xff))

RIO_IP = ipaddress.ip_address('10.63.57.2')
UDP_PORT = 5800

parser = argparse.ArgumentParser(prog="Measurements", description=
"""Process AprilTag frame and returns the distance (cm) and angle (degrees) from the AprilTag"""
)
parser.add_argument("--debug", default=False, help="Enable debugging output", action="store_true")
parser.add_argument("--ip-addr", default=RIO_IP, type=ipaddress.ip_address)
parser.add_argument("--port", default=UDP_PORT, type=int)
parser.add_argument("--send-emptyframe", default=False, action="store_true")

at_detector = Detector(families='tag16h5',
                       nthreads=4,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

# Parameters gotten from passing in images to
# AnalyzeDistortion.py
Cal_file = "../LogitechC920.json"
parameters = json.load(open(Cal_file))
camera_parameters = [parameters["fx"], parameters["fy"], parameters["cx"], parameters["cy"]]
cameraInUse = 0

# Setting up the camera feed
cap = cv2.VideoCapture(cameraInUse)

#TODO: Delete this when using videocapture
# Setting up camera width and height
#if cameraInUse == 0:
#cap.set(4, 800)
cap.set(4, 360)

while (True):
    ret, frame = cap.read()
    frame = frame[120:, ]
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    image = cv2.equalizeHist(image)
    val, th = cv2.threshold(image, 80, 255, cv2.THRESH_OTSU)
    # Tag size: 0.173m
    tags = at_detector.detect(th, estimate_tag_pose=False, camera_params=camera_parameters, tag_size=0.152)

    validTags = []

    for tag in tags:
        pixelDistanceY = abs(tag.corners[2][1] - tag.corners[1][1])
        degreesY = (pixelDistanceY / 2) * VisionConstants.degreesPerPixel
        radians = (math.pi / 180) * degreesY
        tangent = math.tan(radians)
        if abs(tangent) < 1e-6:
            distance = math.inf
        else:
            distance = (VisionConstants.tagHeightCm / 2) / (tangent)
        roundedDistance = float("{0:.2f}".format(distance))

        if tag.tag_id > 8 or pixelDistanceY < 24 or roundedDistance > 250:
            continue
        else:
            validTags.append((tag.tag_id, roundedDistance, 0))
        for p1, p2 in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            cv2.line(frame,
                     (int(tag.corners[p1][0]), int(tag.corners[p1][1])),
                     (int(tag.corners[p2][0]), int(tag.corners[p2][1])),
                     (255, 0, 255), 2)

        # Get X,Y value of center in np array form
        center = tag.center

        # Draws circle dot at the center of the screen
        cv2.circle(frame, (int(center[0]), int(center[1])), radius=8, color=(0, 0, 255), thickness=-1)

        # If the center is located on the right of the screen, degree calculation  for the right will be done.
        if center[0] > 320:
            # Subtract the center position by 320 to get the distance from the center of the screen to the center of the square. Then multiply by 0.3957 to get degrees per pixel.
            degree = (int(center[0]) - 320) * 0.09328
        # If the center is located on the left of the screen, degree calculation  for the right will be done.
        elif center[0] <= 320:
            # Subtract 320 by the center position to get the distance from the center of the screen to the center of the square. Then multiply by 0.3957 to get degrees per pixel.
            degree = (320 - int(center[0])) * 0.09328
        # In case the tag is not on the screen.
        else:
            continue

    # end for tag
    if not validTags:
        continue

    print(degree)
    print(validTags)

    # Display the resulting frame
    cv2.imshow('Video Feed',frame)
    # cv2.imshow('image',image)

    # The time took to proccess the frame
    endTime = time.monotonic()
    # print(f"{endTime - startTime:.4f}")

    # Waits for a user input to quit the application
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()