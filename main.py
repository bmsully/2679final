#!usr/bin/env python3
import time
from enum import Enum
from drive import MotorDriver
from imutils.video import VideoStream
import numpy as np
import cv2
from dt_apriltags import Detector

# Primary raspberry pi code for driving OODA loop

"""
Primary function to run StATV robot
Using camera to locate air vehicle - use an AprilTag and quadcopter or mount to ceiling
Detect if target in view - if so begin/continue MDP policy, otherwise perform spiral search
"""

class StATV():
    def __init__(self):
        # Measured/Calibrated Values
        self.tagWidth = 0.0645 # measured width of AprilTag, in meters
        fx = 507.453 # Focal X
        fy = 508.720 # Focal Y
        cx = 318.699 # Camera X
        cy = 232.403 # Camera Y
        self.camVector = [fx,fy,cx,cy]
        camMatrix = [[fx,0,cx],[0,fy,cy],[0,0,1]]
        self.camAxis = np.float32(camMatrix)
        # Display font properties
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fScale = 0.45
        self.fColor = (0, 255, 255)
        self.fStroke = 2
        # Display axis properties
        axisLength = 0.07
        self.axis = axisLength * np.float32([[1,0,0], [0,1,0], [0,0,-1]])
        # Motor, Video, Detector
        self.motorDriver = MotorDriver()
        self.videoStream = VideoStream(src=0).start()
        self.tagDetector = Detector(families="tag16h5")

    def observe(self):
        frame = self.videoStream.read()
        return frame
    
    def orient(self, frame):
        frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.tagDetector.detect(frameGray, estimate_tag_post=True, camera_params=self.camVector, tag_size=self.tagWidth)

        tagVisible = False
        tagDistanceToCenter = 10000
        for tag in tags:
            if tag.hamming == 0:
                tagVisible = True

                startX = np.int0(min(tag.corners[:,0]))
                startY = np.int0(min(tag.corners[:,1]))
                endX = np.int0(max(tag.corners[:,0]))
                endY = np.int0(max(tag.corners[:,1]))
                centerX = (endX+startX)/2
                centerY = (endY+startY)/2
                cv2.drawContours(frame, np.int0([tag.corners]), 0, self.fColor, 2)
                y = startY-10
                text = "Tag %s" % (tag.tag_id)
                cv2.putText(frame, text, (startX, y), self.font, self.fScale, self.fColor, self.fStroke)
                center_tuple = tuple([int(q) for q in tag.center])

                imgpts,jac = cv2.projectPoints(self.axis, tag.pose_R, tag.pose_t, self.camAxis, np.array([]))
                frame = self.draw(frame, center_tuple, imgpts)
        return (tagVisible, tagDistanceToCenter)

    def draw(frame_, corner, points):
        corner = tuple(np.int0(corner))
        cv2.line(frame_, corner, tuple(np.int0(points[0,0])), (0,0,255), 2)
        cv2.line(frame_, corner, tuple(np.int0(points[1,0])), (0,255,0), 2)
        cv2.line(frame_, corner, tuple(np.int0(points[2,0])), (255,0,), 2)
        return frame_

    def runSpiral(self):
        pass

    def runMDP(self):
        pass

    def driveMotors(self, right=False, left=False):
        print("DRIVE: {r}{l}".format(r="R" if right else "-", l="L" if left else "-"))
        pass

    def stop(self):
        self.motorDriver.stop()
        self.videoStream.stop()

class State(Enum):
    LOST = 0
    SPIRAL = 1
    MDP = 2
    FOUND = 3


def runFSM(logging=False):
    LOOP_TIME = 0.5

    statv = StATV()

    last_state = State.LOST
    state = State.LOST
    try:
        while True:
            start = time.time()
            frame = statv.observe() # Get camera frame
            tagVisible, tagDistance = statv.orient(frame) # check if tag in frame or out

            
            # if logging: print(f"start time: {start}")
            # OBSERVE
            # take picture from camera

            # ORIENT
            # decide LOST, INRANGE, or FOUND
            # if LOST: SPIRAL
            # if INRANGE: MDP
            # if FOUND: STAY

            # DECIDE
            # IF SPIRAL: Set spiral distance (need previous state)
            # IF INRANGE: Run MDP for calculation
            # IF STAY: Do nothing for loop

            # ACT
            # Act accordingly by driving motor (this should probably be a blocking function to avoid queueing movements)
            if state == State.LOST:

                pass
            elif state == State.SPIRAL:
                if last_state == State.SPIRAL:
                    # wider activation
                    pass
                else:
                    #reset activation to 0
                    pass

            elif state == State.MDP:
                if last_state == State.MDP:
                    pass
                else:
                    #reset MDP
                    pass

            elif state == State.FOUND:
                # Turn on extra gpio activated led pin
                # 
                pass


            end = time.time()
            last_state = state
            while end-start < LOOP_TIME:
                end = time.time()


            # key = cv2.waitKey(1) & 0xFF
            # if key == ord('q'):
            #     break
    finally:
        statv.stop()



if __name__ == "__main__":
    runFSM()

