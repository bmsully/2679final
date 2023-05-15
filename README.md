# 2679 Final Project

Brady Sullivan, Spring 2023

## StATV

StATV is a Surface-to-Air Tracking Vehicle designed and built for the Spring 2023 2.679 final project.

The robot is built using a Raspberry Pi 4 with camera, a motor controller PiHat breakout board, and a dual motor plant.

The main code driving StATV can be found in main.py and the motor controller abstraction in drive.py.

Other files:

- calibrate.py: cv2 camera calibration
- cameraCalibration.py: rough duplicate of camera calibration (for automating the process of calibrating and running robot)
- detectAprilTag.py: individual code for AprilTag detection
- oldDrive.py: as it sounds, an original of the motor driver, pre-abstraction
- viewer.py: a module to view camera feed; can apply masking and filtering

## Future Iterations

- Implement MDP policy making and execution code
- Fix imutils/cv2 frames, camera frames were not appearing :(
- Implement more robust logging procedures
- Stream logging and camera frames to external device
- Better abstractions for FSM & StATV functionalties
- Improve "smoothness" of motor control with P or PI control
- Implement function to change target radius based on tag distance detection i.e. smaller tolerance for higher target
- Optimize looping period to minimize robot idling ()
