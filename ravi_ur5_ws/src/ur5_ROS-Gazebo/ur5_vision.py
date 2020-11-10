#!/usr/bin/env python

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
from ur5_notebook.msg import Tracker
import moveit_msgs.msg
import cv2, cv_bridge
from sensor_msgs.msg import Image


from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

#added_by_prem
from scipy.misc import imsave
from vs_main import get_next_pose_from_vs
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Bool

tracker = Tracker()
processing = False
new_msg = False
msg = None
robot_servo_flag = False

def image_callback(data):
    #print('callback.')
    global processing, new_msg, msg
    if not processing:
        new_msg = True
        msg = data
    if new_msg:
        if not robot_servo_flag:
            #set processing to True
            processing = True
            new_msg = False
            try:
                cur_image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, "rgb8")
            except cv_bridge.CvBridgeError as e:
                print(e)
            cv2.imshow("Image window", cur_image)
            cv2.waitKey(3)
            #imshow(msg)

            print('done !!')
            #imsave('target.png',ref_image)
            imsave('cur.png',cur_image)
            next_pose = get_next_pose_from_vs('target.png','cur.png')
            pose_pub.publish(next_pose.astype(np.float32))
            #simulate a process that take 0.2 seconds
            #rospy.loginfo(msg)
            #r.sleep()
            #set processing to False
            processing = False

def servo_callback(flag_msg):
    robot_servo_flag = flag_msg

class ur5_vision:
    def __init__(self):
        rospy.init_node("ur5_vision", anonymous=False)
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.bridge = cv_bridge.CvBridge()
        self.robot_tracker = rospy.Subscriber('robot_servo',Bool, servo_callback)
        self.image_sub = rospy.Subscriber('/ur5/usbcam/image_raw', Image, image_callback)

        global pose_pub
        pose_pub = rospy.Publisher('pose', numpy_msg(Floats), queue_size=1)

        #global ref_image
        #ref_image = cv2.imread('ref_image.png',cv2.IMREAD_COLOR)
        #ref_image = cv2.cvtColor(ref_image, cv2.COLOR_BGR2RGB)


        #while not rospy.is_shutdown():
                #print('listener3.')


    def image_callback(self,msg):


        # BEGIN BRIDGE
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # END BRIDGE
        # BEGIN HSV

        h, w, d = image.shape
        # print h, w, d  (800,800,3)
        #BEGIN FINDER

        cv2.namedWindow("window", 1)
        cv2.imshow("window", image )
        cv2.waitKey(1)

follower=ur5_vision()
rospy.spin()
