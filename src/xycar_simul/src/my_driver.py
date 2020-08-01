#!/usr/bin/env python
import cv2
import rospy
from std_msgs.msg import Int32MultiArray
import time

cap = cv2.VideoCapture('/home/seiya/catkin_ws/src/xycar_simul/track-s.mkv')

# FRAMES PER SECOND FOR VIDEO
fps = 15

def pub_motor(Angle, Speed):
    drive_info = [Angle, Speed]
    drive_info = Int32MultiArray(data = drive_info)
    pub.publish(drive_info)
 
def start():
    global pub
    rospy.init_node('my_driver')
    pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
    rate = rospy.Rate(30)
    Speed = 20

    Angle = -50

    while True:

        
       pub_motor(Angle, Speed) 
       rate.sleep()

if __name__ == '__main__':

    start()
