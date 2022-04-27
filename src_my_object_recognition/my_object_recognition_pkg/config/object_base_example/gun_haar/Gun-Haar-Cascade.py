import cv2
import numpy as np
watch_cascade=cv2.CascadeClassifier('classifier/cascade.xml')

#cap=cv2.VideoCapture(0)
cap=cv2.VideoCapture('Guns Slideshow.mp4')
while True:
    ret,img=cap.read()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    watch = watch_cascade.detectMultiScale(gray,1.3,3)
    for(x,y,w,h) in watch:
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray=gray[y:y+h,x:x+w]
        roi_color=img[y:y+h,x:x+w]
        
    cv2.imshow('img',img)
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break
cap.release()
cv2.destroyAllWindows()

