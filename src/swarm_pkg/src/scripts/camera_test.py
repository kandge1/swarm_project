import cv2
import numpy as np
cap = cv2.VideoCapture())
print(“press ‘q’ to quit”)
while(True):
    ret, frame = cap.read()
    cv2.imshow(‘frame’, frame)
    if cv2.waitKey(1) & 0xFF == ord(‘q’):
        break
cap.release()
cv2.destroyAllWindows()
