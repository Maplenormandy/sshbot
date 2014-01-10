import numpy as np
import cv2

cap = cv2.VideoCapture(1)
cap.set(3,320)
cap.set(4,240)

while(True):

    ret, orig = cap.read()  
    frame = orig.copy()
    blur = cv2.blur(frame, (10,10))
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        
    lower_red1 = np.array([0,255*0.4,255*0.4])
    upper_red1 = np.array([10,255*0.95,255*0.95])
    
    lower_red2 = np.array([170,255*0.50,255*0.50])
    upper_red2 = np.array([180,255*0.73,255*0.84])
    
    src1 = cv2.inRange(hsv, lower_red1, upper_red1)
    src2 = cv2.inRange(hsv, lower_red2, upper_red2)          
    combined = cv2.bitwise_or(src1, src2)
    
    contours,hierarchy = cv2.findContours(combined.copy(), 1, 2)
    
    for s in contours:
      area = cv2.contourArea(s)
      if area > 100:
        print area
        M = cv2.moments(s)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        r = (area / 3.14)**0.5
        cv2.circle(frame, (cx, cy), int(r), (0, 255, 0), 1)
        print cx, cy
    # M = cv2.moments(cnt)
    # contours, hierarchy = cv2.findContours(src1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # contourImg = frame.copy()
    # cv2.drawContours(contourImg, contours, -1, (0,255,0), 3)
    # cv2.circle(frame, (0, 0), 10, (0, 255, 0), -1)
    cv2.imshow('frame1', blur)
    cv2.imshow('frame2', combined)
    cv2.imshow('frame3', frame)
    
    if cv2.waitKey(200) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()