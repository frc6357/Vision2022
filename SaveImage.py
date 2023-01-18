import cv2

cap = cv2.VideoCapture(0)
count = 0

while(True):

    ret, frame = cap.read()
    count += 1

    cv2.imshow('frame', frame)

    # Waits for a user input to quit the application
     

    if cv2.waitKey(1) & 0xFF == ord('s'):
        print("saving image")
        cv2.imwrite(str(count)+'.jpg', frame)



# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()