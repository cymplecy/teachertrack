import cv2
import numpy as np
import time
cam = cv2.VideoCapture(0) #Index in brackets determine the camera. If there is only one camera available the index must be 0
cam.set(3,320) #set width to 320
cam.set(4,240) #set height to 240
while True:
        starttime = time.time()
        img2 = cam.read()[1] # get the camera image
        blur = cv2.blur(img2,(3,3)) # blur it
        #hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV) # convert to HSV colourspace
        #thresh = cv2.inRange(hsv,np.array((0, 90, 90)), np.array((5, 255, 255))) # detect Raspberry Pi Red
        thresh = cv2.inRange(blur,np.array((230, 230, 230)), np.array((255, 255, 255))) # IR
        
        contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        # finding contour with maximum area and store it as best_cnt
        max_area = 0
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if ((area > max_area) and (area > 100)):
                print max_area
                max_area = area
                best_cnt = cnt

        # finding centroids of best_cnt and draw a circle there
        if (best_cnt != None):
            M = cv2.moments(best_cnt)
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            cv2.circle(img2,(cx,cy),5,255,-1)
        cv2.imshow("Detect",thresh)
        cv2.imshow("Video",img2)
        if cv2.waitKey(1) == 27: # Stop if ESC is pressed
                break
        print time.time() - starttime