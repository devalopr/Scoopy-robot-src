#!/usr/bin/env python

import rospy

import rospy
import actionlib

import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Behaviour:

    #Subscribing and publishing init
    def __init__(self) -> None:


        self.way_points = {}

        self.way_points["sink_pose"] = [-2.837,-1.963,-3.124]
        self.way_points["center_pose"] = [-2.405,-1.248,1.580]
        self.way_points["exit_pose"] = [-0.008, -1.395,3.094]

        #Init movebase
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        #pass
        self.behave()


    def behave(self):
        self.move_location("sink_pose")
        self.move_location("center_pose")
        self.move_location("exit_pose")

        #pass


    def move_location(self,location_name):
        map_coord = self.way_points[location_name]
        #print (self.way_points['sink_pose'])
        self.send_movebase_pose(map_coord[0],map_coord[1],map_coord[2])

    #Sending goal to move base
    def send_movebase_pose(self,x,y,theta):

        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y

        quat = quaternion_from_euler (0, 0,theta)
        self.goal.target_pose.pose.orientation.x = quat[0]
        self.goal.target_pose.pose.orientation.y = quat[1]
        self.goal.target_pose.pose.orientation.z = quat[2]
        self.goal.target_pose.pose.orientation.w = quat[3]


        self.client.send_goal(self.goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()



        pass

    #Moving each joint
    def move_joint(self,joint_val):
        pass

    #Return centroid of the detected color
    def detect_object(self,color):
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('scoopy_behave')
        rospy.loginfo("Initializing Scoopy Behaviour")        
        behave_obj = Behaviour()
        

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")