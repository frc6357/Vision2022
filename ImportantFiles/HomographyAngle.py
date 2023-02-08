
"""Calculates the homographical matrix of the image
    2/08/23 Justin Garrity"""

#TODO: Change the class to make it compatible with tag16h5 family
#TODO: Correct the angle and make sure it matches real life

from cmath import tan
import cv2
import time
import numpy as np
import math
from pupil_apriltags import Detector
from _Constants import VisionConstants




at_detector = Detector(families='tag36h11',
                       nthreads=16,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

# Parameters gotten from passing in images to 
# AnalyzeDistortion.py
camera_parameters = [629.45235321,  # fx
                     761.58031703, # fy
                     292.80595879, # cx
                     524.85259558] # cy

# Index of Camera used (0 for pc and 1 for secondary camera)
cameraInUse = 0

#TODO: Not currently implemented, but would increase accuracy if implemented
# Function returns focal length of the camera
def focalLengthPx(imageCenter, corner):
    # Get X,Y value of center in np array form 
    center = imageCenter

    pixelDistanceY =  (corner[2][1] - corner[1][1])

    degreesY = (pixelDistanceY/2) * VisionConstants.degreesPerPixel

    radians = (math.pi / 180) * degreesY

    distance = (VisionConstants.tagHeightPx/2) / (math.tan(radians))

    roundedDistance = float("{0:.2f}".format(distance))
    
        
    return -roundedDistance

# Setting up the camera feed
cap = cv2.VideoCapture(cameraInUse)

while(True):
    ret, frame = cap.read()
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Tag size: 0.173m
    tags = at_detector.detect(image, estimate_tag_pose=True, camera_params=camera_parameters, tag_size=0.173)

    
    for tag in tags:
        for p1, p2 in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            cv2.line(frame,
                    (int(tag.corners[p1][0]), int(tag.corners[p1][1])),
                    (int(tag.corners[p2][0]), int(tag.corners[p2][1])),
                    (255, 0, 255), 2)
        
        h = tag.homography
        angle = math.atan2(h[1,0], h[0,0])
        degreeangle = angle * (180/ math.pi)
        print(degreeangle)

    # Display the resulting frame
    cv2.imshow('Video Feed',frame)

    # The time took to proccess the frame
    endTime = time.monotonic()

    # Waits for a user input to quit the application
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()
