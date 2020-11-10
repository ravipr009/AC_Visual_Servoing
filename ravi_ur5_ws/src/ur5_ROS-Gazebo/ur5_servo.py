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
from geometry_msgs.msg import Twist
import moveit_msgs.msg
from sensor_msgs.msg import Image
from ur5_notebook.msg import Tracker
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from time import sleep


from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
tracker = Tracker()



class ur5_mp:
    def __init__(self):
        rospy.init_node("ur5_mp", anonymous=False)
        rospy.loginfo("hi, is this the init ?: yes")


        self.pose_sub = rospy.Subscriber('pose', numpy_msg(Floats), self.pose_callback, queue_size=1)
        #self.movesig_sub = rospy.Subscriber('movesig', numpy_msg(Floats), self.pose_callback, queue_size=1)
        self.servo_flag = rospy.Publisher('robot_servo',Bool, queue_size=1)

        self.phase = 1
        self.object_cnt = 0
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.points=[]
        self.state_change_time = rospy.Time.now()

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "/base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.1)
        self.arm.set_max_acceleration_scaling_factor(.5)
        self.arm.set_max_velocity_scaling_factor(.5)

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # Initialize the waypoints list
        self.waypoints= []
        self.pointx = []
        self.pointy = []
        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)

        # Set the next waypoint to the right 0.5 meters

        wpose.position.x = -0.2
        wpose.position.y = -0.2
        wpose.position.z = 0.3
        self.waypoints.append(deepcopy(wpose))

        # wpose.position.x = 0.1052
        # wpose.position.y = -0.4271
        # wpose.position.z = 0.4005
        #
        # wpose.orientation.x = 0.4811
        # wpose.orientation.y = 0.5070
        # wpose.orientation.z = -0.5047
        # wpose.orientation.w = 0.5000

        # self.waypoints.append(deepcopy(wpose))


        if np.sqrt((wpose.position.x-start_pose.position.x)**2+(wpose.position.y-start_pose.position.y)**2 \
            +(wpose.position.z-start_pose.position.z)**2)<0.1:
            rospy.loginfo("Warning: target position overlaps with the initial position!")

        # self.arm.set_pose_target(wpose)




        # Specify default (idle) joint states
        self.default_joint_states = self.arm.get_current_joint_values()
        self.default_joint_states[0] = -1.57691
        self.default_joint_states[1] = -1.71667
        self.default_joint_states[2] = 1.79266
        self.default_joint_states[3] = -1.67721
        self.default_joint_states[4] = -1.5705
        self.default_joint_states[5] = 0.0

        self.arm.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan()

        self.arm.execute(plan)

        # Specify end states (drop object)
        self.end_joint_states = deepcopy(self.default_joint_states)
        self.end_joint_states[0] = -3.65
        # self.end_joint_states[1] = -1.3705

        self.transition_pose = deepcopy(self.default_joint_states)
        self.transition_pose[0] = -3.65
        self.transition_pose[4] = -1.95

        print('moved')

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def pose_callback(self, msg):
        rospy.loginfo("HK")
        print(msg)
        pose = msg.data
        pose = pose.astype(np.double)
        self.servo_flag.publish(True)
        #do something here to move the robot
        #todo########################
        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
        wpose = deepcopy(start_pose)

        wpose.position.x = pose[0]
        wpose.position.y = pose[1]
        wpose.position.z = pose[2]

        q = quaternion_from_euler(pose[3],pose[4],pose[5])
        wpose.orientation.x = q[0]
        wpose.orientation.y = q[1]
        wpose.orientation.z = q[2]
        wpose.orientation.w = q[3]

        self.waypoints = []
        self.waypoints.append(deepcopy(wpose))

        self.arm.set_start_state_to_current_state()
        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
        #self.arm.execute(plan)


        self.servo_flag.publish(False)







mp=ur5_mp()
rospy.loginfo("hi, is this the start")

rospy.spin()
