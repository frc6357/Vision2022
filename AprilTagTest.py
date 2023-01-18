import cv2
import time
import numpy as np
from pupil_apriltags import Detector

at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

camera_parameters = [443.6319712,  # fx
                     391.50381628, # fy
                     959.49982957, # cx
                     539.49965467] # cy

cap = cv2.VideoCapture(1)

while(True):
    # Capture frame-by-frame
    startTime = time.monotonic()
    ret, frame = cap.read()

    # Convert image from RGB format to HSV
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # # Threshold of blue in HSV space
    # lower_bound = np.array([0, 0, 0])
    # upper_bound = np.array([255, 50, 255])
 
    # # preparing the mask to overlay
    # mask = cv2.inRange(image, lower_bound, upper_bound)
    # image = cv2.bitwise_and(image, image, mask=mask)

    image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Tag size: 0.173m
    tags = at_detector.detect(image, estimate_tag_pose=True, camera_params=camera_parameters, tag_size=0.173)



    for tag in tags:
        for p1, p2 in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            cv2.line(frame,
                    (int(tag.corners[p1][0]), int(tag.corners[p1][1])),
                    (int(tag.corners[p2][0]), int(tag.corners[p2][1])),
                    (0, 255, 0), 2)
        print(tag)

    # Display the resulting frame
    cv2.imshow('preview',frame)
    # cv2.imshow('image',image)

    endTime = time.monotonic()
    # print(f"{endTime - startTime:.4f}")

    # Waits for a user input to quit the application
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()