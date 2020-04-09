import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(1):
    _, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 100, 200)
    
    dot = cv2.imread('img/dot.jpg', 0)
    w, h = dot.shape[::-1]
    
    cv2.imshow('Source', edges)
    
    # search for dots
    res = cv2.matchTemplate(edges, dot, cv2.TM_CCORR_NORMED)
    threshold = 0.5
    loc = np.where(res >= threshold)
    
    matches = []
    
    #https://stackoverflow.com/questions/21829469/
    mask = np.zeros(edges.shape[:2], np.uint8)
    
    # draw rects over dots and eliminate duplicates
    for pt in zip(*loc[::-1]):
        cv2.rectangle(edges, pt, (pt[0] + w, pt[1] + h), (255,255,255), 2)
        
        if mask[int(pt[1] + h/2), int(pt[0] + w/2)] != 255:
            mask[pt[1]:pt[1]+h, pt[0]:pt[0]+w] = 255
            matches.append(pt)
            
    die_pos = []
    x = 0
    y = 0
    
    # find dice centerpoint by averaging the dots
    if (len(matches)):
        for pt in matches:
            x += pt[0]
            y += pt[1]
        
        die_pos.append(x / len(matches))
        die_pos.append(y / len(matches))
        print("die pos = " + str(die_pos))
        
    cv2.imshow('Detected', edges)
    
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()