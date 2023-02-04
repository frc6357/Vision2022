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

cameraInUse = 0

# Setting up the camera feed
cap = cv2.VideoCapture(cameraInUse)

#TODO: Delete this when using videocapture 
# Setting up camera width and height
#if cameraInUse == 0:
#cap.set(4, 800)
#cap.set(4, 600)
while(True):
    ret, frame, = cap.read()
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Tag size: 0.173m
    tags = at_detector.detect(image, estimate_tag_pose=True, camera_params=camera_parameters, tag_size=0.173)
    output = 0
    
    for tag in tags:
        for p1, p2 in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            cv2.line(frame,
                    (int(tag.corners[p1][0]), int(tag.corners[p1][1])),
                    (int(tag.corners[p2][0]), int(tag.corners[p2][1])),
                    (255, 0, 255), 2)
        #pts1 = np.float([[]])
        
        corner1 = [tag.corners[0][0], tag.corners[0][1]] 
        corner2 = [tag.corners[1][0], tag.corners[1][1]]
        corner3 = [tag.corners[2][0], tag.corners[2][1]]
        corner4 = [tag.corners[3][0], tag.corners[3][1]]
        
        length = 173
        
        pts1 = np.float32([corner1, corner2, corner3, corner4])    
        #pts12 = np.float32([corner3, corner4, corner1, corner2])    
        
        
        
        
        pts2 =  np.float32([[0,0], [length, 0], [0,length], [length, length]])
        
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        
        output = cv2.warpPerspective(frame, matrix, (length, length))
        
        #for x in range(0,4):
        
        
        
        
        
    
        
        
        

    # Display the resulting frame
    cv2.imshow('Video Feed',output)
    cv2.imshow('asdf',frame)
    #cv2.imshow('Unchanged Feed', unChanged)
    # cv2.imshow('image',image)

    # The time took to proccess the frame
    endTime = time.monotonic()
    # print(f"{endTime - startTime:.4f}")

    # Waits for a user input to quit the application
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()
