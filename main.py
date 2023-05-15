#!usr/bin/env python3
import time
from enum import Enum
from drive import MotorDriver
from imutils.video import VideoStream
import numpy as np
import cv2
from dt_apriltags import Detector
import RPi.GPIO as GPIO
import threading

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
        tags = self.tagDetector.detect(frameGray, estimate_tag_pose=True, camera_params=self.camVector, tag_size=self.tagWidth)

        tagVisible = False
        tagDistanceToCenter = 10000
        angle = 0
        for tag in tags:
            if tag.hamming == 0:
                tagVisible = True

                startX = np.int0(min(tag.corners[:,0]))
                startY = np.int0(min(tag.corners[:,1]))
                cv2.drawContours(frame, np.int0([tag.corners]), 0, self.fColor, 2)
                y = startY-10
                text = "Tag %s" % (tag.tag_id)
                cv2.putText(frame, text, (startX, y), self.font, self.fScale, self.fColor, self.fStroke)
                center_tuple = tuple([int(q) for q in tag.center])
                tagDistanceToCenter = ((center_tuple[0]-320)**2+(center_tuple[1]-240)**2)**0.5
                if tagDistanceToCenter == 0:
                    angle = 0
                else:
                    vCenterToTag = [(center_tuple[0]-320)/tagDistanceToCenter, (center_tuple[1]-240)/tagDistanceToCenter]
                    vForward = [0,-1]
                    angle = np.degrees(np.arccos(np.dot(vCenterToTag, vForward)))
                if center_tuple[0] < 320:
                    angle = -1*angle

                imgpts,jac = cv2.projectPoints(self.axis, tag.pose_R, tag.pose_t, self.camAxis, np.array([]))
                frame = self.draw(frame, center_tuple, imgpts)
        return (tagVisible, tagDistanceToCenter, angle, frame)

    def draw(self, frame_, corner, points):
        corner = tuple(np.int0(corner))
        cv2.line(frame_, corner, tuple(np.int0(points[0,0])), (0,0,255), 2)
        cv2.line(frame_, corner, tuple(np.int0(points[1,0])), (0,255,0), 2)
        cv2.line(frame_, corner, tuple(np.int0(points[2,0])), (255,0,), 2)
        return frame_

    def runSpiral(self, duration):
        driveForward = threading.Thread(target=self.driveMotors, args=(duration, True, True, ))
        driveForward.start()
        driveForward.join()
        # self.driveMotors(duration, right=True, left=True)
        self.driveMotors(0.4, right=True, left=False)

    def runMDP(self):
        nan = np.nan
        actions = [[],[]]
        # P = np.array([[[],[],[],[]],[[],[],[],[]],])
        # R = np.array([])
        # Need to explore more positional encoding for better reward function
        # Modify below to capture actions during policy
        Q = np.full((_,_), -np.inf)
        for s, a in enumerate(actions):
            Q[s,a] = 0.0
        discount_factor = 0.99
        iterations = 10
        for i in range(iterations):
            Q_previous = Q.copy()
            for s in range(len(P)):
                for a in actions[s]:
                    sum_v = 0
                    for s_next in range(len(P)):
                        sum_v += P[s, a, s_next] * (R[s, a, s_next] + discount_factor * np.max(Q_previous[s_next]))
                    Q[s,a] = sum_v
        print(Q)
        # execute motor control based on policy
        
    def backupPlan(self, distance, last_distance, angle, last_angle):
        if angle < -10:
            # drive left
            driveLeft = threading.Thread(target=self.driveMotors, args=(0.3, False, True, ))
            driveLeft.start()
            driveLeft.join()
        elif angle > 10:
            #drive right
            driveRight = threading.Thread(target=self.driveMotors, args=(0.3, True, False, ))
            driveRight.start()
            driveRight.join()
        # drive forward regardless
        driveForward = threading.Thread(target=self.driveMotors, args=(0.3, True, True, ))
        driveForward.start()
        driveForward.join()
        
        

    def driveMotors(self, duration, right=False, left=False):
        print("DRIVE: {r}{l} for {d} seconds".format(r="R" if right else "-", l="L" if left else "-", d=duration))
        if right and left:
            self.motorDriver.driveBoth(duration)
            pass
        elif right and not left:
            self.motorDriver.driveRight(duration)
            pass
        elif not right and left:
            self.motorDriver.driveLeft(duration)
            pass
        elif not right and not left:
            # Do nothing
            pass
        else:
            print("Unexpected state - should not reach")

    def stop(self):
        self.motorDriver.stop()
        self.videoStream.stop()

class State(Enum):
    LOST = 0
    INRANGE = 1
    FOUND = 2


def runFSM(logging=False):
    LOOP_TIME = 0.5
    CENTER_TOLERANCE = 50

    statv = StATV()
    GPIO.setup(16, GPIO.OUT)
    GPIO.output(16, GPIO.LOW)
    
    driveDuration = 0.1
    last_angle = 0
    angle = 0
    last_distanceToCenter = 1000
    distanceToCenter = 1000

    last_state = State.FOUND
    state = State.LOST
    try:
        while True:
            start = time.time()
            if logging: print(f"start time: {start}")
            # OBSERVE - take picture from camera
            frame = statv.observe() # Get camera frame
            if logging: print("OBSERVE: IMAGE CAPTURED")
            # ORIENT - decide LOST=>SPIRAL, INRANGE=>MDP, or FOUND=>STAY
            tagVisible, distanceToCenter, angle, frame = statv.orient(frame) # check if tag in frame or out
            cv2.circle(frame, (320,240), CENTER_TOLERANCE, (0, 255, 0), 2)
            cv2.imshow("frame", frame)
            print(tagVisible, distanceToCenter, angle)
            if tagVisible:
                if distanceToCenter < CENTER_TOLERANCE:
                    state = State.FOUND
                    if logging: print("ORIENT: FOUND")
                else:
                    state = State.INRANGE
                    if logging: print("ORIENT: INRANGE")
            else:
                state = State.LOST
                if logging: print("ORIENT: LOST")
            # DECIDE - LOST=>Set spiral distance, INRANGE=>Run MDP, STAY=>No action
            # ACT - Drive motor accordingly
            if state == State.LOST:
                GPIO.output(16, GPIO.LOW)
                if last_state == State.LOST:
                    driveDuration += 0.1
                else:
                    driveDuration = 0.1
                if logging: print("DECIDE: SPIRAL DURATION SET")
                if logging: print("ACT: SPIRAL")
                statv.runSpiral(driveDuration)

            elif state == State.INRANGE:
                GPIO.output(16, GPIO.LOW)
                if last_state == State.INRANGE:
                    pass
                else:
                    #reset MDP
                    pass
                # if logging: print("DECIDE: MDP POLICY SEARCH")
                # if logging: print("ACT: EXECUTING POLICY")
                # statv.runMDP()
                if logging: print("DECIDE: BACKUP PLAN")
                if logging: print("ACT: EXECUTE BACKUP POLICY")
                statv.backupPlan(distanceToCenter, last_distanceToCenter, angle, last_angle)

            elif state == State.FOUND:
                # Turn on extra gpio activated led pin
                if logging: print("DECIDE: NO ACTION")
                if logging: print("ACT: STATIONARY")
                GPIO.output(16, GPIO.HIGH)
                
            else:
                print("Unexpected state, should not reach")

            last_distanceToCenter = distanceToCenter
            last_angle = angle
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
    time.sleep(30)
    runFSM(True)

