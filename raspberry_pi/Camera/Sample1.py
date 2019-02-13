import cv2

c = cv2.VideoCapture(0) #1
c.set(3,320)
c.set(4,240)

r, img = c.read() #2
cv2.imwrite('capture.jpg', img) #3
c.release()
