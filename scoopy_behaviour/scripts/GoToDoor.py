#!/usr/bin/env python
import rospy
import roslaunch
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import rospkg


class Go:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.initial_theta = 0
        self.front_scan = 200
        self.angle_scan = 0
        self.scan = []
        self.odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)        
        self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.tool_head_topic = "/scoopy/toolhead_revolute_position_controller/command"
        self.tool_head_joint = rospy.Publisher(self.tool_head_topic, Float64, queue_size=10)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        rospack = rospkg.RosPack()
        nav_pkg_path = rospack.get_path('scoopy_navigation')+str("/launch/bathroom_bringup.launch")

        rospy.loginfo("Navigation launch file path"+nav_pkg_path)

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [nav_pkg_path])
        self.main()
        


    def main(self):
        rospy.sleep(2)
        self.tool_head_joint.publish(-1.57)
        self.go_to_door()
        print("LAUNCHING NAVIGATION STACK...")
        self.launch.start()
        rospy.spin()
        
    def go_to_door(self):
        
        min_scan = 100
        vel = Twist()
        vel.angular.z = 0.3
        self.cmd_vel.publish(vel)
        self.initial_theta = self.theta
        #rospy.sleep(2)
        t0 = rospy.get_time()
        
        while not rospy.is_shutdown():                                  #Scanning to find the wall
            print("Theta = ", self.theta, "scan = ", self.front_scan) 
            t_now = rospy.get_time()
            delta_theta = abs(self.initial_theta - self.theta)
            delta_t = rospy.get_time() - t0
            if ((delta_t > 3) and (delta_theta < 0.1)):
                print("delta_t = ", delta_t, " delta_theta = ", delta_theta, " SCANNED")
                vel.angular.z = 0
                self.cmd_vel.publish(vel)
                break
            else:
                vel.angular.z = 0.3
                self.cmd_vel.publish(vel)
            if self.front_scan < min_scan:
                min_scan = self.front_scan
                min_scan_angle = self.theta
            
            rospy.sleep(0.1)
            
        
        print("Initial_Theta = ", self.initial_theta)
        print("Theta = ", self.theta) 
        print("min_scan = ", min_scan)
        print("min_scan_angle = ", min_scan_angle) 
        rospy.sleep(1)
        
        while not rospy.is_shutdown():                        #aligning the robot perpendicular to the wall
            delta_theta = abs(min_scan_angle - self.theta)
            print("delta_theta =", delta_theta) 
            if delta_theta > 0.1:
                vel.angular.z = 0.5
                self.cmd_vel.publish(vel)
            else:
                vel.angular.z = 0
                self.cmd_vel.publish(vel)
                break
            
            rospy.sleep(0.1)
        

        print("ANGLE COARSE ALIGNED")
        print("Theta = ", self.theta) 
                
        while not rospy.is_shutdown():                         #positioning the robot 0.8m from the wall
            if 0.8 - self.front_scan > 0.05:
                vel.linear.x = -0.2
            elif 0.8- self.front_scan < -0.05:
                vel.linear.x = 0.2
            else:
                vel.linear.x = 0
                self.cmd_vel.publish(vel)
                break
            
            self.cmd_vel.publish(vel)
            rospy.sleep(0.1)             

        print("DISTANCE ALIGNED")
        print("Distance = ", self.front_scan)
        
        min_scan = 100
        t0 = rospy.get_time()
        vel.angular.z = 0.1
        while not rospy.is_shutdown():                          #precisely aligning the robot to the wall
            delta_t = rospy.get_time() - t0
            delta_theta = abs(min_scan_angle - self.theta)
            if (delta_t > 6.5) and (delta_theta < 0.1): 
                vel.angular.z = 0
                self.cmd_vel.publish(vel)
                break
            elif delta_t > 6: vel.angular.z = 0.1
            elif delta_t > 2: vel.angular.z = -0.1

            if self.front_scan < min_scan:
                min_scan = self.front_scan
                min_scan_angle = self.theta
                print("new min, theta = ", self.theta, "front_scan = ", self.front_scan)
            
            rospy.sleep(0.1)
            self.cmd_vel.publish(vel)

        print("ANGLE FINE ALIGNED")
        print("min_scan_angle = ", min_scan_angle, " delta_theta = ", delta_theta)
        rospy.sleep(1)

        door_reached = False
        while not rospy.is_shutdown():                          #loops till the door is detected 

            while not rospy.is_shutdown():                      #moves the robot to the edge of the wall
                if self.front_scan > 1.5:
                    if self.angle_scan > 2:       
                        print("REACHED WALL END")
                        rospy.sleep(2)                              #delay for the robot go past the end of the wall by ~1m
                        vel.linear.y = 0 
                        self.cmd_vel.publish(vel)
                        break
                    else:
                        door_reached = True
                        print("REACHED DOOR")
                        rospy.sleep(0.5)
                        vel.linear.y = 0 
                        self.cmd_vel.publish(vel)
                        break  
                elif (self.front_scan > 0.2):
                    vel.linear.y = -0.5
                    self.cmd_vel.publish(vel)  

                rospy.sleep(0.1)
                print("front_scan = ", self.front_scan, "theta = ", self.theta)

            
            
            if door_reached == True:
                break
            
            if min_scan_angle < 1.57:
                min_scan_angle = min_scan_angle + 1.57
            else:
                min_scan_angle = -3.14

            while not rospy.is_shutdown():                        #turns perpendicular to the next wall
                delta_theta = abs(min_scan_angle - self.theta)
                print("delta_theta =", delta_theta) 
                if delta_theta > 0.1:
                    vel.angular.z = 0.5
                    self.cmd_vel.publish(vel)
                else:
                    vel.angular.z = 0
                    self.cmd_vel.publish(vel)
                    break

                rospy.sleep(0.1)
            
            vel.linear.y = -0.5 
            self.cmd_vel.publish(vel)
            rospy.sleep(3)
            vel.linear.y = 0 
            self.cmd_vel.publish(vel)
            rospy.sleep(1)
            
            if door_reached == True:
                break

    

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def scan_callback(self, msg):
        self.front_scan = msg.ranges[360]
        self.angle_scan = msg.ranges[280]
        self.scan = msg

if __name__ == '__main__':
    rospy.init_node('test', anonymous = True)
    Go()
    
   