#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped,Pose,PoseArray
from nav_msgs.msg import Path
from squaternion import Quaternion
print('Path generator node')

rospy.init_node('Path_node')

topic='path'
pub=rospy.Publisher(topic, PoseArray, queue_size=10)

myPosearray=PoseArray()
myPosearray.header.frame_id = "neck"
k=5
for i in range(-2000,4000,1):
    x=20+i/100
    if k>0:
        k-=0.001
    #y=-0.2*(pow(x-5,2)+4)
    #tangent=-0.2*2*pow(x-5,1)
    y=k*2*math.sin(0.3*x)
    tangent=k*2*0.3*math.cos(0.3*x)
    Theta=math.atan(tangent)
    print(x,y,Theta)
    myPose=Pose()
    myPose.position.x=x
    myPose.position.y=y
    myPose.position.z=0
    quat=Quaternion.from_euler(0, 0, Theta)
    print(quat)
    myPose.orientation.x=quat[1]
    myPose.orientation.y=quat[2]
    myPose.orientation.z=quat[3]
    myPose.orientation.w=quat[0]
    print(myPose.orientation)
    myPosearray.poses.append(myPose)

rate = rospy.Rate(1) # 10hz
while not rospy.is_shutdown():
    pub.publish(myPosearray)
    #print('publishing path')
    rate.sleep()
