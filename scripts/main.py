#!/usr/bin/python
# -*- coding: utf-8 -*-

# [Import]------------------------------->
import sys
import time
import json
import math
import types
import numpy as np 

import rospy
import rospkg
from std_msgs.msg import Int32MultiArray
from ros_openpose_joint_converter.msg import Joints
from openpose_ros_msgs.msg import Persons

# reference <--- https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md
parts_id_pair = [
        [ 0,  1,  2],
        [ 1,  2,  3],
        [ 2,  3,  4],
        [ 0,  1,  5],
        [ 1,  5,  6],
        [ 5,  6,  7],
        [ 1,  0, 14],
        [ 0, 14, 16],
        [ 1,  0, 15],
        [ 0, 15, 17],
        [ 0,  1,  8],
        [ 1,  8,  9],
        [ 8,  9, 10],
        [ 0,  1, 11],
        [ 1, 11, 12],
        [11, 12, 13],
]


# [ClassDefine]-------------------------->
class Converter():
    ''' 
    This class receive human joint angle from openpose.
    '''
    def __init__(self):
        # ROS Param ---------->>>
        base = rospy.get_namespace() + rospy.get_name()
        self.MISSING_VALUE = rospy.get_param(base + '/missing_value')
        # ROS Subscriber ----->>>
        self.openpose_sub = rospy.Subscriber('/openpose/pose', Persons, self.openposeCB)
        # ROS Publisher ------>>>
        self.joint_angle = rospy.Publisher('/joint_angle', Joints, queue_size=1)


    def openposeCB(self, msg):
        ''' 
        @param msg : it is joint angle from openpose.
                     data format depend on ros-openpose.
        ''' 
        vector = self.convertFormat2Vector(msg)
        self.publishPose(vector)


    def publishPose(self, vector):
        output = Joints()
        output.num = vector.shape[0]
        for i in range(output.num):
            array = Int32MultiArray()
            array.data = vector[i,:]
            output.persons.append(array)
        self.joint_angle.publish(output)


    def convertFormat2Vector(self, msg):
        ''' 
        @param msg : ros-openpose format
        @return output : array (16dim)
        ''' 
        output = np.zeros((0, 16))
        for person in msg.persons:
            angles = []
            for p in parts_id_pair:
                x0 = person.body_part[p[0]].x
                y0 = person.body_part[p[0]].y
                x1 = person.body_part[p[1]].x
                y1 = person.body_part[p[1]].y
                x2 = person.body_part[p[2]].x
                y2 = person.body_part[p[2]].y
                if all([x0, x1, x2, y0, y1, y2]):
                    u = np.array( [ x1 - x0, y1 - y0])
                    v = np.array( [ x2 - x0, y2 - y0])
                    angle = np.round(self.innerProduct(u, v))
                    angles.append(angle)
                else:
                    angles.append(self.MISSING_VALUE)

            output = np.vstack((angles, output))
            
        return output


    def innerProduct(self, u, v):
        ''' 
        calcJointAngle
        ''' 
        i = np.inner(u, v)
        n = np.linalg.norm(u) * np.linalg.norm(v)

        c = i / n
        deg = np.rad2deg(np.arccos(np.clip(c, -1.0, 1.0)))
        return deg


# [Main] ----------------------------------------->>>
#if __name__ == '__main__':
rospy.init_node('joint_converter')

time.sleep(3.0)
node = Converter()

while not rospy.is_shutdown():
    rospy.sleep(0.1)
            
