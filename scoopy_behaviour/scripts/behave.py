#!/usr/bin/env python


class Behaviour:

    def __init__(self) -> None:
        pass

    #Sending goal to move base
    def send_movebase_pose(self,x,y,theta):
        pass

    #Moving each joint
    def move_joint(self,joint_val):
        pass

    #Return centroid of the detected color
    def detect_object(self,color):
        pass

