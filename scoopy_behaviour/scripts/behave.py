#!/usr/bin/env python

import rospy

import rospy
import actionlib

import sys

from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry

import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray



class Behaviour:

    #Subscribing and publishing init
    def __init__(self) -> None:

        #Keeping location points
        self.way_points = {}

        self.bridge = CvBridge()

        #RED color thresholding
        self.red_lowerBound=np.array([149,199,69])
        self.red_upperBound=np.array([179,255,255])

        #Dust bin blue color
        self.blue_lowerBound=np.array([11,150,45])
        self.blue_upperBound=np.array([158,220,255])

        #Yellow threshold
        self.yellow_lowerBound=np.array([13,68,195])
        self.yellow_upperBound=np.array([164,126,255])

        #Orange threshold
        self.orange_lowerBound=np.array([0,219,0])
        self.orange_upperBound=np.array([179,255,255])


        self.kernelOpen=np.ones((5,5))
        self.kernelClose=np.ones((20,20))

        self.trash_pose_x = 0
        self.trash_pose_y = 0

        self.scan = []

        self.way_points["entry_pose"] = [-2.23,-0.048,3.14]
        self.way_points["counter_pose"] = [-1.5,-1.5,-1.7]
        self.way_points["sink_pose_right"] = [-2.9,-1.3,-3.14]

        self.way_points["center_pose"] = [-2.405,-1.248,3.14]
        self.way_points["exit_pose"] = [0,0,0]
        self.way_points["exit_pose_prev"] = [-1.5,0,0]


        #Init movebase
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()

        #Marker array
        self.marker_pub = rospy.Publisher("marker_topic", MarkerArray,queue_size=10)  # check if publisher object is used an
        self.markerArray = MarkerArray()


        #Defining publishers
        self.topic_name = {}
        self.topic_name["camera"] = "/real_sense/color/image_raw"
        self.topic_name["blw"] = "/scoopy/blw_revolute_position_controller/command"
        self.topic_name["brw"] = "/scoopy/brw_revolute_position_controller/command"
        #self.topic_name["camera_pan"] = "/scooopy/camera1/image_raw"
        self.topic_name["camera_tilt"] = "/scoopy/camera_tilt_position_controller/command"
        self.topic_name["lid"] = "/scoopy/lid_revolute_position_controller/command"
        self.topic_name["mid_arm"] = "/scoopy/mid_inner_slider_position_controller/command"
        self.topic_name["outer_arm"] = "/scoopy/outer_mid_slider_position_controller/command"
        self.topic_name["post_slider"] = "/scoopy/post_slider_position_controller/command"
        self.topic_name["tool_head"] = "/scoopy/toolhead_revolute_position_controller/command"




        self.post_joint = rospy.Publisher(self.topic_name["post_slider"],Float64,queue_size=10)
        self.outer_joint = rospy.Publisher(self.topic_name["outer_arm"],Float64,queue_size=10)
        self.mid_joint = rospy.Publisher(self.topic_name["mid_arm"],Float64,queue_size=10)
        self.lid_joint = rospy.Publisher(self.topic_name["lid"],Float64,queue_size=10)
        self.tool_head_joint = rospy.Publisher(self.topic_name["tool_head"],Float64,queue_size=10)
        self.camera_tilt_joint = rospy.Publisher(self.topic_name["camera_tilt"],Float64,queue_size=10)

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        
        #Image processing
        self.processed_pub = rospy.Publisher("detected_img",Image,queue_size=10)
        self.image_sub = rospy.Subscriber(self.topic_name["camera"],Image,self.callback)

        
        #self.post_joint = rospy.Publisher(self.topic_name["post_slider"],Float64,queue_size=10)
        #self.post_joint = rospy.Publisher(self.topic_name["post_slider"],Float64,queue_size=10)

        #pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)

        #pass 
        self.behave()


    def behave(self):                              #main function
        self.move_joint("camera_tilt",-1)
        rospy.sleep(4)
        self.move_joint("tool_head", -1.57)
        rospy.sleep(2)
        rospy.loginfo("Moving inside")
        self.move_location("entry_pose")

        rospy.loginfo("Moving to counter-sink")
        self.move_location("counter_pose")
        
        rospy.sleep(1)
        self.align_counter()
        rospy.sleep(1)
        self.clean_counter()
        self.clean_sink()
        rospy.sleep(4)

        #Move to center
        self.move_location("center_pose")

        rospy.sleep(4)
        #Scanning object around the robot      
        self.scan_objects()
        rospy.sleep(4)

        self.move_location("exit_pose_prev")
        self.move_location("exit_pose")


    #Image thresholding for different color
    def color_threshold(self,cv_image,upper,lower):


        imgHSV= cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(imgHSV,upper,lower)
        maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,self.kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,self.kernelClose)
        maskFinal=maskClose
        conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(cv_image,conts,-1,(255,0,0),3)
        for i in range(len(conts)):
            x,y,w,h=cv2.boundingRect(conts[i])
            #print("X:",(2*x+w)/2,"Y:",(2*x+h)/2)
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255), 2)
            self.trash_pose_x = (2*x+w)/2
            self.trash_pose_y = (2*x+h)/2

        #cv2.imshow("Image window",cv_image)
        #cv2.waitKey(3)

        try:
            self.processed_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)



    #Image callback
    def callback(self,data):

        #rospy.loginfo("Recieving images")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            self.color_threshold(cv_image,self.red_lowerBound,self.red_upperBound)
            self.color_threshold(cv_image,self.blue_lowerBound,self.blue_upperBound)
            self.color_threshold(cv_image,self.yellow_lowerBound,self.yellow_upperBound)
            self.color_threshold(cv_image,self.orange_lowerBound,self.orange_upperBound)

        except CvBridgeError as e:
            print(e)
            pass


    def scan_callback(self, msg):
        self.scan = msg


    #Marker function
    def publish_marker(self,x,y):

        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.5                         # can keep this as gobal parameters
        marker.scale.y = 0.5
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker.pose.position.x = x
        marker.pose.position.y = y

        marker.pose.orientation.w = 1.0

        self.markerArray.markers.append(marker)
        self.marker_pub.publish(self.markerArray)


    def clean_counter(self):
        rospy.loginfo("Extending post slider")
        self.move_joint("post_slider", 1.02)
        rospy.sleep(18)
        self.move_joint("arm", 0.6)
        rospy.sleep(8)
        self.move_joint("arm", 0.2)
        rospy.sleep(5)
        self.move_joint("tool_head", 1.57)
        rospy.sleep(3)
        self.move_joint("arm", 0.6)
        rospy.sleep(5)
        self.move_joint("arm", 0.2)
        rospy.sleep(5)
        self.move_joint("tool_head", -1.57)
        rospy.sleep(3)
        self.align_xy(0,0.65)
        self.move_joint("arm", 0.6)
        rospy.sleep(8)
        self.move_joint("arm", 0.2)
        rospy.sleep(6)

    def clean_sink(self):
        self.move_joint("post_slider", 1.06)
        rospy.sleep(5)
        self.align_xy(0,0.85)
        self.move_joint("arm", 0.5)
        rospy.sleep(8)
        self.move_joint("arm", 0.2)
        rospy.sleep(6)
        self.align_xy(0,1.15)
        self.move_joint("tool_head", 0)
        self.move_joint("arm", 0.4)
        rospy.sleep(8)
        self.move_joint("arm", 0)
        rospy.sleep(8)
        self.move_joint("tool_head", -1.57)
        rospy.sleep(3)
        self.move_joint("post_slider", 0)
        rospy.sleep(18)


    def scan_arm(self):

        self.move_joint("tool_head", 0)
        rospy.sleep(4)
        self.move_joint("arm", 0.2)

        self.move_joint("lid", 4.6)
        rospy.sleep(4)

        self.move_joint("lid", 0)
        rospy.sleep(4)

        self.move_joint("arm", 0)
        rospy.sleep(4)

        #self.move_joint("mid_arm", 0)
        #rospy.sleep(4)

        self.move_joint("lid", 0)
        rospy.sleep(4)

    def scan_objects(self):
        rospy.loginfo("Scanning for trash")
        rospy.sleep(3)

        map_coord = self.way_points["center_pose"]
        for i in range(-3,3):
            rospy.sleep(4)
            rospy.loginfo("Rotating Angle>"+str(i))
            rospy.loginfo("Trash Found:pose_x:"+str(self.trash_pose_x)+" pose_y:"+ str(self.trash_pose_y))
            self.send_movebase_pose(map_coord[0],map_coord[1],i)
            self.scan_arm()
       


    #Assume robot is in init state
    def start_cleaning_couter(self):
        rospy.loginfo("Started cleaning couter top")

        rospy.loginfo("Extending post slider")
        self.move_joint("post_slider", 1.02)
        rospy.sleep(20)

        rospy.loginfo("Extending outer arm")
        self.move_joint("outer_arm", 0.32)
        rospy.sleep(6)

        rospy.loginfo("Cleaning using mid arm")

        self.move_joint("mid_arm", 0.32)
        rospy.sleep(3)

        self.move_joint("mid_arm", 0.22)
        rospy.sleep(3)

        #Publish marker
        
        self.publish_marker(-3.5, -2.2)
        self.publish_marker(-3.51, -2.21)
        self.publish_marker(-3.52, -2.23)
        self.publish_marker(-3.53, -2.24)


        self.move_joint("mid_arm", 0.32)
        rospy.sleep(3)

        self.move_joint("mid_arm", 0.22)
        rospy.sleep(3)

        #Publish marker
       
        self.publish_marker(-3.5, -1.9)
        self.publish_marker(-3.51, -1.9)
        self.publish_marker(-3.52, -1.9)
        self.publish_marker(-3.53, -1.9)


        rospy.loginfo("Completed cleaning counter top")

    def align_counter(self):                                              
        vel = Twist()
        while not rospy.is_shutdown():  
            delta_scan = abs(self.scan.ranges[320] - self.scan.ranges[400])
            print("delta_scan = ", delta_scan)
            if delta_scan > 0.007:
                vel.angular.z = 0.1
                self.cmd_vel.publish(vel)  
            else:
                vel.angular.z = 0
                self.cmd_vel.publish(vel)
                break
            rospy.sleep(0.1)
        
        self.align_xy(0.27,0)
        rospy.sleep(0.5)
        self.align_xy(0,0.35)
        
    def align_xy(self,x,y):
        vel = Twist()
        if x > 0:
            while not rospy.is_shutdown():  
                delta_scan = self.scan.ranges[360] - x 
                if delta_scan > 0.01:
                    vel.linear.x = 0.1
                    self.cmd_vel.publish(vel)  
                else:
                    vel.linear.x = 0
                    self.cmd_vel.publish(vel)
                    break
                rospy.sleep(0.1)
        
        if y > 0:
            while not rospy.is_shutdown():  
                delta_scan = self.scan.ranges[540] - y 
                print("y = ", y, " delta_scan = ", delta_scan)
                if delta_scan > 0.01:
                    vel.linear.y = 0.1
                    self.cmd_vel.publish(vel)  
                elif delta_scan < -0.01:
                    vel.linear.y = -0.1
                    self.cmd_vel.publish(vel)  
                else:
                    vel.linear.y = 0
                    self.cmd_vel.publish(vel)
                    break
                rospy.sleep(0.1)
        



    def move_location(self,location_name):
        rospy.loginfo("Moving into: "+location_name)

        map_coord = self.way_points[location_name]
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
            pass
        else:
            return self.client.get_result()
            pass




    #Moving each joint
    def move_joint(self,name,joint_val):

        if(name == "post_slider"):
            self.post_joint.publish(joint_val)

        elif(name == "arm"):
            self.outer_joint.publish(joint_val/2)
            self.mid_joint.publish(joint_val/2) 

        elif(name == "lid"):
            self.lid_joint.publish(joint_val)

        elif(name == "tool_head"):
            self.tool_head_joint.publish(joint_val)
        elif(name == "camera_tilt"):
            self.camera_tilt_joint.publish(joint_val)    

    #Return centroid of the detected color
    def detect_object(self,color):
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('scoopy_behave')
        rospy.loginfo("Initializing Scoopy Behaviour")  
        while True:
            rospy.loginfo("After pressing start button in Gazebo, Press any key to start the challenge")
            input_data = cv2.waitKey(30)   
            if(input_data):
                break        
        behave_obj = Behaviour()
        

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")