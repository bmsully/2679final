from imutils.video import VideoStream
import numpy as np
import cv2
from dt_apriltags import Detector

def draw(frame_, corner, points):
    corner = tuple(np.int0(corner))
    cv2.line(frame_, corner, tuple(np.int0(points[0,0])), (0,0,255), 2)
    cv2.line(frame_, corner, tuple(np.int0(points[1,0])), (0,255,0), 2)
    cv2.line(frame_, corner, tuple(np.int0(points[2,0])), (255,0,), 2)
    return frame_

vs = VideoStream(src=0).start()

at_detector = Detector(families="tag16h5")

tagWidth = 0.0645 # measured width of your Apriltag, in meters

axisLength = 0.07
axis = axisLength * np.float32([[1,0,0], [0,1,0], [0,0,-1]])

# is there a way/need to automate this? perhaps write to a text file, check for values, then if not present, begin 
fx = 507 # measured values
fy = 508 
cx = 318
cy = 232

camVector = [fx,fy,cx,cy]
camMatrix = [[fx,0,cx],[0,fy,cy],[0,0,1]]
camAxis = np.float32(camMatrix)

font = cv2.FONT_HERSHEY_SIMPLEX
fScale = 0.45
fColor = (0, 255, 255)
fStroke = 2

while True:
    frame = vs.read()
    frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(frameGray, estimate_tag_pose=True, camera_params=camVector, tag_size=tagWidth)

    for tag in tags:
        if tag.hamming == 0 and tag.tag_id == 19:
            startX = np.int0(min(tag.corners[:,0]))
            startY = np.int0(min(tag.corners[:,1]))
            cv2.drawContours(frame, np.int0([tag.corners]), 0, fColor, 2)
            y = startY-10
            text = "Tag %s" % (tag.tag_id)
            cv2.putText(frame, text, (startX, y), font, fScale, fColor, fStroke)
            center_tuple = tuple([int(q) for q in tag.center])
            print(((center_tuple[0]-320)**2+(center_tuple[1]-240)**2)**0.5)

            imgpts,jac = cv2.projectPoints(axis, tag.pose_R, tag.pose_t, camAxis, np.array([]))
            frame = draw(frame, center_tuple, imgpts)
    cv2.circle(frame, (320,240), 75, (0, 255, 0), 2)
    cv2.imshow("frame", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
vs.stop()

cv2.destroyAllWindows()
