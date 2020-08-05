#!/usr/bin/env python
import cv2
import rospy
from std_msgs.msg import Int32MultiArray
import time
from algorithms import *

# object which stores info of track-s.mkv
cap = cv2.VideoCapture('/home/seiya/catkin_ws/src/xycar_simul/track-s.mkv')

# FRAMES PER SECOND FOR VIDEO
fps = 30

def pub_motor(Angle, Speed):
    drive_info = [Angle, Speed]
    drive_info = Int32MultiArray(data = drive_info)
    pub.publish(drive_info)
 
def start():
    global pub
    rospy.init_node('my_driver')
    pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
    rate = rospy.Rate(30)
    Speed = 5

    Angle = ""

    while True:
        # Read the video file.
        ret, frame = cap.read()

        # If we got frames, show them.
        if ret == True:
            imgFinal, imgFinalDuplicate = Perspective(frame, pts1)
            histogramLane = Histogram(imgFinalDuplicate)
            LeftLanePos, RightLanePos = LaneFinder(imgFinal, histogramLane)
            Result = LaneCenter(imgFinal, LeftLanePos, RightLanePos)
            
            if Result == 0:
                Angle = 0
            
            elif 0 < Result < 5:
                Angle = 5

            elif 5 <= Result < 10:
                Angle = 12

            elif 10 <= Result < 15:
                Angle = 17

            elif 15 <= Result:
                Angle = 20

            elif -5 < Result < 0:
                Angle = -5

            elif -10 < Result <= -5:
                Angle = -12

            elif -15 < Result <= -10:
                Angle = -17

            elif Result <= -15:
                Angle = -20

            # Display the frame at same frame rate of recording
            # Watch lecture video for full explanation
            time.sleep(1/fps)
            cv2.imshow('frame',frame)

            # Press q to quit
            if cv2.waitKey(25) & 0xFF == ord('q'): 
            # if cv2.waitKey(25) == ord('q'):
                break

        # Or automatically break this whole loop if the video is over.
        else:
            break

        pub_motor(Angle, Speed) 
        rate.sleep()
    
    cap.release()

if __name__ == '__main__':

    start()
