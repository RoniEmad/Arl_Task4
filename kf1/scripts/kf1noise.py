#!/usr/bin/env python
import time
import math,random
import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped,Pose,PoseArray,Pose2D
from visualization_msgs.msg import Marker
from navigation_task_one.msg import Inputs,Outputs
from squaternion import Quaternion
import matplotlib.pyplot as plt
print('noise node')
rospy.init_node('noise')
topic1 = 'observation'
mark = rospy.Publisher(topic1, Outputs, queue_size=10)
topic2 = 'observationarray'
pub=rospy.Publisher(topic2, PoseArray, queue_size=10)

myPosearray=PoseArray()
myPosearray.header.frame_id = "neck"

time_nanoseclast=time.time_ns()

stateX=[]
stateY=[]
stateTheta=[]
noisyX=[]
noisyY=[]
noisyTheta=[]
def randsign():
    return 1 if random.random() < 0.5 else -1
def saturateangle(th):
    if th>(2*math.pi):
        #print('up ',th)
        th=th-(2*math.pi)
        #print(th)
        return th
    elif th<0:
        #print('down ',th)
        th+=(2*math.pi)
        #print(th)
        return th
    else:
        #print('noo ',th)
        return th
def callback(data):
    
    global time_nanosec,time_nanoseclast,firsttime
    time_nanosec = time.time_ns()
    #print(time_nanosec)
    dt=(time_nanosec-time_nanoseclast)*10**-8
    #print(dt)
    X=data.X
    Y=data.Y
    Theta=data.Theta
    stateX.append(X)
    stateY.append(Y)
    stateTheta.append(Theta)
    noiX=X+np.random.normal()/50+0.000009*dt
    noiY=Y+np.random.normal()/50+0.000009*dt
    noiTheta=saturateangle(Theta+np.random.normal()*math.pi/18000*10+0.000009*dt)
    noisyX.append(noiX)
    noisyY.append(noiY)
    noisyTheta.append(noiTheta)
    mark.publish(Outputs(noiX,noiY,noiTheta))
    myPose=Pose()
    myPose.position.x=noiX
    myPose.position.y=noiY
    myPose.position.z=0
    quat=Quaternion.from_euler(0, 0, noiTheta)
    #print(quat)
    myPose.orientation.x=quat[1]
    myPose.orientation.y=quat[2]
    myPose.orientation.z=quat[3]
    myPose.orientation.w=quat[0]
    #print(myPose.orientation)
    myPosearray.poses.append(myPose)
    pub.publish(myPosearray)


def subscriber():
    rospy.Subscriber('position', Outputs, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
