import cv2
import time
import numpy as np
from pupil_apriltags import Detector

at_detector = Detector(families='tag36h11',
                       nthreads=16,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

# Parameters gotten from passing in images to 
# AnalyzeDistortion.py
camera_parameters = [443.6319712,  # fx
                     391.50381628, # fy
                     959.49982957, # cx
                     539.49965467] # cy

# Setting up the camera feed
cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    startTime = time.monotonic()
    ret, frame = cap.read()

    # Convert image from RGB format to Grayscale
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Tag size: 0.173m
    tags = at_detector.detect(image, estimate_tag_pose=True, camera_params=camera_parameters, tag_size=0.173)

    # Operate on each individual tag found by the detector
    for tag in tags:

        # Drawing a boxe around the found tag
        for p1, p2 in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            cv2.line(frame,
                    (int(tag.corners[p1][0]), int(tag.corners[p1][1])),
                    (int(tag.corners[p2][0]), int(tag.corners[p2][1])),
                    (255, 0, 255), 2)

        # Extracting the rotation matrix from the tag data
        rotation = np.array(tag.pose_R)

        # Getting the <0, 0, 1> vector components multiplied by
        # the rotation matrix
        rotated_x = rotation[0][2]
        rotated_y = rotation[1][2]
        rotated_z = rotation[2][2]

        # Calculates the vertical angle the center of the camera
        # makes relative to the april tag
        pitch = np.rad2deg(np.arctan(rotated_y / rotated_z))

        # Calculates the horizontal angle the center of the camera
        # makes relative to the april tag
        yaw = np.rad2deg(np.arctan(rotated_x / rotated_z))

        print("X Angle: ", yaw)
        print("Y Angle: ", pitch)

    # Display the resulting frame
    cv2.imshow('Video Feed',frame)
    # cv2.imshow('image',image)

    # The time took to proccess the frame
    endTime = time.monotonic()
    # print(f"{endTime - startTime:.4f}")

    # Waits for a user input to quit the application
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
