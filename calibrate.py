from imutils.video import VideoStream
import numpy as np
import cv2
import time

objp = np.zeros((6*4, 3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:4].T.reshape(-1,2)

objpoints = []
imgpoints = []
i = 0

vs = VideoStream(src=0).start()

while True:
	frame = vs.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	ret, corners = cv2.findChessboardCorners(gray, (6,4), None)
	
	if ret == True:
		i = i+1
		objpoints.append(objp)
		imgpoints.append(corners)
		cv2.drawChessboardCorners(frame, (6,4), corners, ret)
	cv2.imshow("frame", frame)
	cv2.waitKey(500)
	if i > 20:
		break
vs.stop()
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(mtx)
cv2.destroyAllWindows
