import cv2
resolution = [400,400]
cam = cv2.VideoCapture(0)
cam.set(3,resolution[0])
cam.set(4,resolution[1])
ret,image=cam.read()
